"""
RLTaskOptimiser — bridges the ROS2 node and the RL task sequencer.

Subclasses TaskSequencerEvaluator and adds coordinate conversion between
robot space (physical metres) and RL grid space (training scale):

  Robot space:  grid 0.075 m, x in [-0.45, 0.45], y in [0.00, 0.45]
  RL space:     grid 0.75  m, x in [-4.5,  4.5],  y in [0.00, 4.5]
  Scale factor: RL = robot × 10

Pick pose:  original raw robot-space XYZW preserved for perception accuracy.
Place pose: RL output cell centre ÷ 10, z = PLACE_Z, identity orientation.
"""

import os
import sys

# Locate rl_task_optimiser/ relative to this file.
# Installed layout: lib/hl_control/rl_task_optimiser/  (same directory as this script)
# Source layout:    hl_control/rl_task_optimiser/       (one level up from hl_control/)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_RL_INSTALLED = os.path.join(_SCRIPT_DIR, 'rl_task_optimiser')
_RL_SOURCE    = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', 'rl_task_optimiser'))
_RL_ROOT = _RL_INSTALLED if os.path.isdir(_RL_INSTALLED) else _RL_SOURCE

if _RL_ROOT not in sys.path:
    sys.path.insert(0, _RL_ROOT)

from task_sequencer import TaskSequencerEvaluator          # noqa: E402
from reward import custom_reward                           # noqa: E402
from training_env.wordle_env import ALL_POSITIONS, WORKSPACE_X_MIN  # noqa: E402

_MODELS_INSTALLED = os.path.join(_SCRIPT_DIR, 'models')
_MODELS_SOURCE    = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', 'models'))
_MODELS_DIR = _MODELS_INSTALLED if os.path.isdir(_MODELS_INSTALLED) else _MODELS_SOURCE
MODEL_NAME = "wordle_ppo"

ROBOT_SCALE = 10.0      # multiply robot coords by this to enter RL space
GRID_STEP_RL = 0.75     # RL grid cell size in metres
PLACE_Z = 0.025         # fixed z for all place poses in robot space


class RLTaskOptimiser(TaskSequencerEvaluator):
    """
    Subclass of TaskSequencerEvaluator that adds robot↔RL coordinate conversion.

    Instantiate once at node startup. Call solve() for each word request.
    """

    def __init__(self, model_path: str | None = None):
        if model_path is None:
            model_path = os.path.join(_MODELS_DIR, f"{MODEL_NAME}_latest")
        super().__init__(model_path=model_path, reward_callback=custom_reward)

        # cell_id → original raw robot-space pose, populated per solve() call.
        self._raw_pick_poses: dict[int, tuple] = {}

    # ------------------------------------------------------------------
    # Public interface called by HLControlNode
    # ------------------------------------------------------------------

    def solve(
        self,
        target_word: str,
        letters: list[dict],
    ) -> list[dict]:
        """
        Run the RL model and return a task sequence ready for ROS publishing.

        Parameters
        ----------
        target_word : str
            Five-letter word to spell (upper-case).
        letters : list of dict
            Each dict has keys:
              'letter'    : str    — single upper-case letter
              'object_id' : str    — e.g. 'C_object_1'
              'x', 'y', 'z': float — raw robot-space position
              'qx', 'qy', 'qz', 'qw': float — raw robot-space orientation

        Returns
        -------
        list of dict — each entry has:
          'step'        : int
          'description' : str
          'pick_x', 'pick_y', 'pick_z': float  — original robot-space pick
          'pick_qx', 'pick_qy', 'pick_qz', 'pick_qw': float
          'place_x', 'place_y': float           — RL-derived, robot space
          'place_z'    : float                  — always PLACE_Z
          'object_id'  : str
          'letter'     : str
          'source_cell_id', 'dest_cell_id': int
        """
        self._raw_pick_poses = {}
        perception_blocks: dict[int, str] = {}

        for item in letters:
            cell_id = self._robot_to_cell_id(item['x'], item['y'])
            perception_blocks[cell_id] = item['letter']
            self._raw_pick_poses[cell_id] = item

        env = self.build_env(stage=3, word=target_word, fixed_positions=perception_blocks)
        trajectory = self.run_episode(env)
        raw_sequence = self.get_task_sequence(trajectory)

        return self._enrich_sequence(raw_sequence, letters)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _robot_to_cell_id(x_robot: float, y_robot: float) -> int:
        """Snap a robot-space (x, y) to the nearest RL grid cell_id."""
        x_rl = x_robot * ROBOT_SCALE
        y_rl = y_robot * ROBOT_SCALE
        col = round((x_rl - WORKSPACE_X_MIN) / GRID_STEP_RL)
        row = round(y_rl / GRID_STEP_RL)
        col = max(0, min(12, col))
        row = max(0, min(6, row))
        return row * 13 + col

    @staticmethod
    def _rl_to_robot(x_rl: float, y_rl: float) -> tuple[float, float]:
        """Convert RL-space (x, y) to robot-space (x, y)."""
        return x_rl / ROBOT_SCALE, y_rl / ROBOT_SCALE

    # ------------------------------------------------------------------
    # Sequence enrichment
    # ------------------------------------------------------------------

    def _enrich_sequence(
        self,
        raw_sequence: list[dict],
        letters: list[dict],
    ) -> list[dict]:
        """
        Attach robot-space pick/place poses and object_ids to each step.

        Pick pose  → original raw robot-space pose from perception.
        Place pose → RL cell centre converted to robot space.
        """
        cell_to_item: dict[int, dict] = {}
        for item in letters:
            cid = self._robot_to_cell_id(item['x'], item['y'])
            cell_to_item[cid] = item

        current_cell_to_item: dict[int, dict] = dict(cell_to_item)

        enriched = []
        for step in raw_sequence:
            src_id = step['source_cell_id']
            dst_id = step['dest_cell_id']

            item = current_cell_to_item.get(src_id)
            if item is None:
                item = {'x': 0.0, 'y': 0.0, 'z': PLACE_Z,
                        'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
                        'letter': step['letter'], 'object_id': ''}

            dst_x_rl, dst_y_rl = ALL_POSITIONS[dst_id]
            place_x, place_y = self._rl_to_robot(dst_x_rl, dst_y_rl)

            enriched.append({
                'step':           step['step'],
                'description':    step['description'],
                'pick_x':         item['x'],
                'pick_y':         item['y'],
                'pick_z':         item['z'],
                'pick_qx':        item.get('qx', 0.0),
                'pick_qy':        item.get('qy', 0.0),
                'pick_qz':        item.get('qz', 0.0),
                'pick_qw':        item.get('qw', 1.0),
                'place_x':        place_x,
                'place_y':        place_y,
                'place_z':        PLACE_Z,
                'object_id':      item.get('object_id', ''),
                'letter':         step['letter'],
                'source_cell_id': src_id,
                'dest_cell_id':   dst_id,
            })

            current_cell_to_item[dst_id] = item
            current_cell_to_item.pop(src_id, None)

        return enriched
