"""
RLTaskOptimiser — wraps TaskSequencerEvaluator for ROS2 deployment.

Handles all coordinate conversion between robot space and RL model space:
  - Robot space: grid 0.075 m, x in [-0.45, 0.45], y in [0.0, 0.45]
  - RL space:    grid 0.75  m, x in [-4.5,  4.5],  y in [0.0, 4.5]
  - Scale factor: RL = robot * 10

Pick pose:  original raw robot-space XYZW is preserved for accuracy.
Place pose: RL output cell centre / 10, with z=PLACE_Z and identity orientation.
"""

import os
import sys

import train as _train_module
from test import TaskSequencerEvaluator
from train import custom_reward, MODEL_DIR, MODEL_NAME
from training_env.wordle_env import ALL_POSITIONS, WORKSPACE_X_MIN

# Root of the rl_task_optimiser package (where train.py lives).
_RL_ROOT = os.path.dirname(os.path.abspath(_train_module.__file__))

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
            model_path = os.path.join(_RL_ROOT, MODEL_DIR, f"{MODEL_NAME}_latest")
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
        letter_counters: dict[str, int] = {}

        for item in letters:
            cell_id = self._robot_to_cell_id(item['x'], item['y'])
            perception_blocks[cell_id] = item['letter']
            self._raw_pick_poses[cell_id] = item
            letter_counters[item['letter']] = letter_counters.get(item['letter'], 0) + 1

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
        # Build cell_id → letter item lookup for pick resolution
        cell_to_item: dict[int, dict] = {}
        for item in letters:
            cid = self._robot_to_cell_id(item['x'], item['y'])
            cell_to_item[cid] = item

        # Track which cells have been visited (pick moves the letter)
        current_cell_to_item: dict[int, dict] = dict(cell_to_item)

        enriched = []
        for step in raw_sequence:
            src_id = step['source_cell_id']
            dst_id = step['dest_cell_id']

            item = current_cell_to_item.get(src_id)
            if item is None:
                # Letter was placed into this cell by a previous step — it keeps
                # the same raw pose it was put there with (staging moves).
                item = {'x': 0.0, 'y': 0.0, 'z': PLACE_Z,
                        'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
                        'letter': step['letter'], 'object_id': ''}

            # Place pose: convert RL cell centre to robot space
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

            # Update the tracking map: item moves from src to dst
            current_cell_to_item[dst_id] = item
            current_cell_to_item.pop(src_id, None)

        return enriched
