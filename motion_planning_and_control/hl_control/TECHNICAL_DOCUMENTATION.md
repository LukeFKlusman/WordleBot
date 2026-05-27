# hl_control Technical Documentation

## Project overview

`hl_control` is the high-level task sequencing package for WordleBot. It receives a requested five-letter word and the current gameboard state, runs a trained MaskablePPO policy to decide the ordered pick-and-place moves, and publishes those moves to `wordlebot_control` for physical execution.

This page focuses on the package internals and design decisions. Full robot bringup, complete launch ordering, and system-level screenshots belong in the full-stack overview page.

## Key features

- Passive ROS 2 node that waits for both a word request and a fresh board state.
- MaskablePPO model integration inside a ROS node.
- Conversion between physical robot coordinates and the RL model's grid coordinates.
- Action masking through the Gymnasium environment so the model only selects valid symbolic moves.
- Sequential task-list generation from the model trajectory.
- MoveIt collision object publication for every detected letter tile.
- `PickPlaceTask` publication to the low-level `wordlebot_control` package.
- Isolated test publisher for validating the HL package without perception, solver, or robot hardware.

## Dependencies

`hl_control` is a ROS 2 Humble Python package with custom messages and a bundled RL inference engine.

Important runtime dependencies:

| Dependency | Purpose |
| --- | --- |
| `rclpy` | ROS 2 Python node, publishers, subscribers, parameters. |
| `std_msgs` | Receives the target word as `std_msgs/String`. |
| `geometry_msgs` | Carries perceived letter poses and generated pick/place poses. |
| `moveit_msgs` | Publishes `CollisionObject` messages for letter cubes. |
| `shape_msgs` | Defines the 50 mm letter collision cube primitive. |
| `wordlebot_control` | Provides the `PickPlaceTask` message consumed by low-level control. |
| `stable-baselines3` | Base RL tooling. |
| `sb3-contrib` | Provides `MaskablePPO`. |
| `gymnasium` | RL environment API used by `WordleSequencingEnv`. |
| `numpy`, `matplotlib`, `PyYAML` | RL evaluation, plotting, and isolated YAML test input. |

Hardware assumptions inherited by the generated tasks:

- Letter tiles are represented as 50 mm cubes.
- Perception publishes letter poses in the `world` frame.
- The physical board uses a 0.075 m grid.
- The five Wordle target slots are centred at `y=0.225 m` and `x={-0.150,-0.075,0.000,0.075,0.150}`.
- `wordlebot_control` is responsible for arm/gripper planning and execution; `hl_control` never commands robot joints directly.

## Package structure

| File | Role |
| --- | --- |
| `hl_control/hl_control_node.py` | ROS node. Receives inputs, triggers solve, publishes collision objects and tasks. |
| `hl_control/rl_task_optimiser.py` | Bridge between robot-space perception data and the RL task sequencer. |
| `rl_task_optimiser/task_sequencer.py` | Loads the MaskablePPO checkpoint, runs inference, converts trajectories to symbolic tasks. |
| `rl_task_optimiser/training_env/wordle_env.py` | Gymnasium environment, board state, action mask, observation builder, symbolic transition logic. |
| `rl_task_optimiser/reward.py` | Reward function used by training/evaluation and inference environment construction. |
| `models/wordle_ppo_latest.zip` | Default trained MaskablePPO checkpoint. |
| `msg/LetterObject.msg` | One perceived tile: letter, pose, object ID. |
| `msg/GameboardState.msg` | Array of perceived letter tiles. |
| `launch/hl_control.launch.py` | Starts `hl_control_node.py` and passes the model path. |
| `config/tc2_1_board.yaml` | Example isolated test board. |
| `test/test_sim.py` | Publishes a test word and board state into ROS. |
| `test/demonstration_test.py` | Standalone non-ROS model/evaluator demonstration. |

## Runtime interface

The node name is `hl_control_node`.

### Subscribed topics

| Topic | Type | QoS | Meaning |
| --- | --- | --- | --- |
| `/hl_control/word_request` | `std_msgs/msg/String` | `TRANSIENT_LOCAL`, depth 1 | Requested five-letter word. The node strips whitespace and converts to upper-case. |
| `/perception/gameboard_state` | `hl_control/msg/GameboardState` | `TRANSIENT_LOCAL`, depth 1 | Current detected board state: all visible letter tiles, IDs, and poses. |

Latched QoS is a design decision: the node can start after the solver or perception publisher and still receive the most recent word/board message. This is useful during integration where nodes may be launched in different terminals.

### Published topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `/wordle_bot/add_collision_object` | `moveit_msgs/msg/CollisionObject` | Adds/removes letter collision cubes in the low-level MoveIt scene. |
| `/perception/letter_objects` | `wordlebot_control/msg/PickPlaceTask` | Publishes one pick-and-place task per planned model action. |

`hl_control` deliberately does not publish `/wordle_bot/start_mission`. It queues tasks into `wordlebot_control`, then an operator, UI, or full-stack coordinator starts execution. This keeps decision-making separate from robot actuation.

### Message definitions

`hl_control/msg/LetterObject.msg`

```text
string letter
geometry_msgs/PoseStamped pose
string object_id
```

`hl_control/msg/GameboardState.msg`

```text
hl_control/LetterObject[] letters
```

`wordlebot_control/msg/PickPlaceTask.msg`

```text
geometry_msgs/PoseStamped pick_pose
geometry_msgs/Pose place_pose
string object_id
```

## System role

`hl_control` sits between the game/perception layer and the low-level robot controller:

```text
Word request + perceived board
        |
        v
HLControlNode
        |
        v
RLTaskOptimiser
        |
        v
TaskSequencerEvaluator + WordleSequencingEnv + MaskablePPO
        |
        v
ordered task dictionaries
        |
        v
PickPlaceTask messages + collision objects
        |
        v
wordlebot_control
```

The important design boundary is that `hl_control` only decides "which tile moves where, and in what order". It does not plan trajectories, solve IK, open/close the gripper, or recover from physical failures. Those behaviours belong to `wordlebot_control`.

## Input-to-task pipeline

This is the core package flow from gameboard state and word request to the sequential task list.

### 1. Word request callback

`HLControlNode._word_callback()` receives `/hl_control/word_request`.

It:

1. Strips whitespace.
2. Converts the word to upper-case.
3. Rejects anything that is not exactly five alphabetic characters.
4. Stores the result in `_pending_word`.
5. Clears `_board_letters` so the next solve requires a fresh board scan.
6. Calls `_try_solve()`.

Clearing `_board_letters` is intentional. A new word should not be solved against an old perception snapshot because the physical board may have changed after the previous request.

### 2. Board state callback

`HLControlNode._board_callback()` receives `/perception/gameboard_state`.

For every `LetterObject`, it builds a plain Python dictionary:

```text
{
  letter,
  object_id,
  x, y, z,
  qx, qy, qz, qw
}
```

The callback normalises the letter to upper-case and guarantees each object ID is unique. If perception sends a duplicate or empty ID, the node creates a fallback ID such as `C_object_1` or appends a numeric suffix. This matters because MoveIt collision objects and `PickPlaceTask.object_id` must refer to the same physical tile.

After storing the board list in `_board_letters`, the callback calls `_try_solve()`.

### 3. Solve trigger

`HLControlNode._try_solve()` only runs the model when both pieces of input exist:

```text
_pending_word is not None
_board_letters is not None
```

When both are available:

1. It copies the word and board list.
2. It clears `_pending_word` to prevent repeated solves from the same pair.
3. It publishes letter collision objects into the MoveIt scene.
4. It calls `self.solve(word, letters)`.
5. It logs the returned ordered sequence.
6. It publishes one `PickPlaceTask` per step.

The node is therefore event-driven rather than timer-driven. It avoids repeatedly re-solving the same board state and only acts when the integration layer provides new information.

### 4. Collision scene publication

Before solving, `_add_letters_to_scene()` publishes MoveIt collision objects for all detected letters:

- Current letter IDs are compared with `_scene_object_ids`.
- Stale IDs from the previous board are removed with `CollisionObject.REMOVE`.
- Current letters are added as 50 mm box primitives in the `world` frame.
- The collision object pose uses the perceived pose directly.

This happens in `hl_control` because the high-level node has the complete detected board state. It lets `wordlebot_control` plan with the same letter objects that the model used for sequencing.

### 5. Robot-space board to RL grid

`RLTaskOptimiser.solve()` converts the perceived board into the symbolic format expected by the RL model.

The physical board is:

```text
13 columns x 7 rows
cell spacing = 0.075 m
x range = -0.45 m to +0.45 m
y range =  0.00 m to +0.45 m
```

The RL model was trained on the same grid scaled by 10:

```text
cell spacing = 0.75
x range = -4.5 to +4.5
y range =  0.0 to +4.5
```

For each perceived tile:

```text
x_rl = x_robot * 10
y_rl = y_robot * 10
col = round((x_rl - WORKSPACE_X_MIN) / 0.75)
row = round(y_rl / 0.75)
cell_id = row * 13 + col
```

The result is `perception_blocks`, a dictionary mapping grid cell IDs to letters:

```text
{cell_id: letter}
```

The raw robot-space poses are kept separately. This is a key design decision: the model should reason on the grid, but the robot should pick using the highest-fidelity perception pose rather than the snapped grid centre.

### 6. RL environment construction

`RLTaskOptimiser.solve()` calls:

```python
env = self.build_env(stage=3, word=target_word, fixed_positions=perception_blocks)
```

`build_env()` creates a `WordleSequencingEnv` with:

- `target_word`: the requested word.
- `fixed_initial_positions`: the perceived grid cells.
- `stage=3`: the deployment action-mask mode.
- `custom_reward`: the reward function used to evaluate symbolic moves.

Stage 3 is used in deployment because it supports both common live-board cases:

- C2-like boards where Wordle slots are empty and correct letters are in staging.
- C3-like boards where some Wordle slots are blocked by wrong letters and must be cleared first.

The `SOLVE_STAGE` constant exists in `hl_control_node.py`, but the current `solve()` implementation passes `stage=3` directly.

### 7. Observation and action model

The RL environment represents the board as a 13 x 7 symbolic grid.

Important constants:

| Item | Value |
| --- | --- |
| Cells | `91` |
| Wordle slot cell IDs | `[43, 44, 45, 46, 47]` |
| Action space | `91 * 91 = 8281` discrete actions |
| Action encoding | `action = source_cell_id * 91 + dest_cell_id` |
| Observation size | `2686` floats |

The observation contains:

- Normalised robot position.
- For each cell: occupied flag, one-hot letter, and Wordle-correct flag.
- Five `needs_clearing` flags for blocked Wordle slots.
- One-hot target word encoding.
- Curriculum stage indicator.

The model is `MaskablePPO`, so it receives an action mask at every step. This is central to the design: the neural policy ranks among currently legal symbolic moves instead of wasting decisions on impossible actions.

In stage 3, valid actions are:

- From a Wordle slot: move a wrong letter to any empty non-forbidden staging cell.
- From staging: move a letter only into its matching empty Wordle slot.
- Correctly placed Wordle letters are never valid sources.

This action mask encodes the task rules that must never be violated physically, while the learned policy chooses the order that minimises cost and completes the word.

### 8. Model inference

`TaskSequencerEvaluator.run_episode()` runs one deterministic model episode:

1. `env.reset()` builds the symbolic board from the fixed perceived positions.
2. `env.action_masks()` computes valid symbolic moves.
3. `MaskablePPO.predict(obs, deterministic=True, action_masks=masks)` selects one action.
4. `env.step(action)` updates the symbolic board.
5. The selected action is recorded in `step_frames`.
6. The loop repeats until the word is complete or the stage step limit is reached.

Each step frame records:

- Source cell ID.
- Destination cell ID.
- Letter moved.
- Robot position before the move.
- Source and destination grid positions.
- Board state before and after the move.

The model therefore outputs a trajectory of symbolic moves, not robot poses. Robot poses are reconstructed in the next layer.

### 9. Trajectory to symbolic task sequence

`TaskSequencerEvaluator.get_task_sequence()` converts the model trajectory into a list of task dictionaries:

```text
{
  step,
  description,
  pick_pose,
  place_pose,
  source_cell_id,
  dest_cell_id,
  letter
}
```

At this stage, `pick_pose` and `place_pose` are still RL-grid XY coordinates from `ALL_POSITIONS`. The sequence is ordered exactly as the model executed it in the environment.

### 10. Sequence enrichment for robot execution

`RLTaskOptimiser._enrich_sequence()` converts symbolic tasks into robot-ready dictionaries.

For each step:

- `source_cell_id` is used to find the current tile dictionary.
- The pick position uses the original raw perceived robot-space `x`, `y`, `z`.
- The pick orientation is converted from a perception yaw quaternion into an end-effector-down quaternion.
- `dest_cell_id` is converted from RL-grid XY back to robot-space XY by dividing by 10.
- The place Z uses the tile's current perceived `z`.
- The `object_id` is copied from the tile dictionary.
- An internal `current_cell_to_item` map is updated so later model moves can pick from locations created by earlier moves.

The internal map update is important for clearing moves. If the model first moves a wrong letter out of a Wordle slot, later steps need to know that the same object now lives in its new staging cell.

### 11. Quaternion design

Perception is assumed to provide yaw-only tile orientation:

```text
(qx, qy, qz, qw) = (0, 0, sin(theta/2), cos(theta/2))
```

The robot gripper must point down. `_perception_to_ee_quat()` preserves yaw while converting to an end-effector-down orientation:

```text
pick orientation = (cos(theta/2), sin(theta/2), 0, 0)
```

This corresponds to roll approximately `pi`, pitch `0`, and the perceived yaw. The design decision is to preserve tile yaw at pick time while enforcing the gripper approach direction needed by the robot.

For place poses, `_publish_tasks()` currently publishes:

```text
place orientation = (x=1.0, y=0.0, z=0.0, w=0.0)
```

This is also an end-effector-down quaternion. The model does not choose place yaw; all place targets use the fixed placement orientation expected by `wordlebot_control`.

### 12. Publishing low-level tasks

`HLControlNode._publish_tasks()` converts each enriched dictionary into a `PickPlaceTask`:

- `pick_pose.header.frame_id = "world"`
- `pick_pose` uses raw perception position and converted pick quaternion.
- `place_pose.position` uses the model-selected destination in robot space.
- `place_pose.orientation` is fixed end-effector-down.
- `object_id` matches the collision object ID.

The messages are published in sequence on `/perception/letter_objects`. The low-level `wordlebot_control` node queues them and waits for `/wordle_bot/start_mission` before executing.

## Design decisions

### Why use RL for sequencing?

A simple hand-written planner can place five letters into five slots, but the task becomes more interesting when there are distractors, blocked Wordle slots, and a desire to reduce total robot travel. The RL policy was trained to choose a valid order while considering travel cost and board constraints.

The model does not replace low-level motion planning. It solves the symbolic ordering problem:

```text
which object -> which destination -> in what order
```

MoveIt still handles:

- IK.
- Collision-aware arm paths.
- Gripper control.
- Cartesian approach/lift/descent/retreat.
- Stop/resume/abort recovery.

### Why action masking?

The action space contains 8281 possible `source -> destination` moves. Most are invalid in any given board state. MaskablePPO allows the environment to remove illegal actions before inference.

This gives two benefits:

- The model does not waste capacity learning that impossible moves are impossible.
- The deployed robot is protected from symbolic actions that violate Wordle task rules, such as moving a correct letter out of its slot or placing a letter into the wrong slot.

### Why keep pick poses raw but place poses grid-based?

Pick poses come from perception and represent the measured physical tile location. Snapping the pick to a grid centre would throw away useful perception accuracy and could cause grasp misses.

Place poses are selected by the model as grid cells. For placement, the grid centre is the correct target because Wordle slots and staging cells are discrete destinations.

### Why publish collision objects here?

The high-level node receives the complete board state, so it is the first package that knows all current tile IDs and poses. Publishing collision objects from here keeps the planning scene consistent with the board state used for model inference. `wordlebot_control` then receives both the collision objects and task messages that refer to those same IDs.

### Why not start the robot automatically?

Publishing tasks and starting execution are separate by design. It allows:

- Operators to inspect the logged task sequence first.
- A UI or game manager to decide when execution begins.
- The low-level controller to queue tasks before motion starts.
- Safer testing of the high-level model without robot movement.

## Coordinate system

Robot-space grid:

| Item | Value |
| --- | --- |
| Columns | 13 |
| Rows | 7 |
| Cell size | `0.075 m` |
| X range | `-0.45 m` to `+0.45 m` |
| Y range | `0.00 m` to `+0.45 m` |
| Robot origin cell | column 6, row 0, cell ID 6 |

RL-space grid:

| Item | Value |
| --- | --- |
| Cell size | `0.75` |
| X range | `-4.5` to `+4.5` |
| Y range | `0.0` to `+4.5` |
| Scale | `RL = robot * 10` |

Wordle slots:

| Slot | Cell ID | Robot X | Robot Y |
| --- | --- | --- | --- |
| 0 | 43 | `-0.150` | `0.225` |
| 1 | 44 | `-0.075` | `0.225` |
| 2 | 45 | `0.000` | `0.225` |
| 3 | 46 | `0.075` | `0.225` |
| 4 | 47 | `0.150` | `0.225` |

## Configurable settings

Runtime launch argument:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model_path` | Installed `models/wordle_ppo_latest` | MaskablePPO checkpoint path without `.zip`. |

Important source constants:

| Constant | File | Value | Meaning |
| --- | --- | --- | --- |
| `LETTER_CUBE_SIZE` | `hl_control_node.py` | `0.05` | Collision cube side length in metres. |
| `SOLVE_STAGE` | `hl_control_node.py` | `3` | Intended deployment stage constant. Current solve path hard-codes stage 3. |
| `ROBOT_SCALE` | `rl_task_optimiser.py` | `10.0` | Robot metres to RL grid scale factor. |
| `GRID_STEP_RL` | `rl_task_optimiser.py` | `0.75` | RL grid cell spacing. |
| `PLACE_Z` | `rl_task_optimiser.py` | `0.025` | Defined fixed place height; current enrichment uses the tile's perceived `z`. |
| `WORDLE_CELL_IDS` | `wordle_env.py` | `[43,44,45,46,47]` | Target slot cell IDs. |
| `MAX_STEPS_PER_STAGE` | `wordle_env.py` | C1 10, C2 15, C3 25, C4 35 | Episode step limits. |

## Running and testing this package

Full stack running belongs in the overview page. For isolated HL validation after the workspace is built and sourced:

```bash
ros2 launch hl_control hl_control.launch.py
```

In another terminal:

```bash
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

Expected behaviour:

- `hl_control_node` logs that it received a board state and word request.
- It publishes letter collision objects to `/wordle_bot/add_collision_object`.
- It logs an ordered task sequence.
- It publishes one `PickPlaceTask` per step to `/perception/letter_objects`.
- It does not move the robot until another node publishes `/wordle_bot/start_mission`.

To inspect generated tasks:

```bash
ros2 topic echo /perception/letter_objects
```

Standalone model/evaluator check without ROS:

```bash
python3 test/demonstration_test.py
```

## Known limitations and assumptions

- The node handles five-letter words only.
- The deployed model path must exist as `<model_path>.zip`.
- Deployment uses stage 3 action masking.
- `SOLVE_STAGE` is currently not used by `RLTaskOptimiser.solve()`; stage 3 is hard-coded there.
- The model chooses symbolic grid cells, not continuous poses.
- The model does not choose place orientation.
- The place pose orientation is fixed to end-effector-down.
- The enrichment path currently uses the tile's perceived `z` as `place_z`, even though `PLACE_Z` is defined as `0.025`.
- The board is snapped to the nearest grid cell. Large perception errors can map a tile to the wrong cell.
- If two physical letters snap to the same cell, the later dictionary entry overwrites the earlier one in `perception_blocks`.
- The node does not receive low-level execution feedback and does not replan after a failed physical pick/place.
- A new word request clears the stored board state and waits for a fresh scan.
- `_pending_word` is cleared after solving; another solve needs another word request.
- Collision objects are simple 50 mm boxes, not detailed tile meshes.

## Troubleshooting

### Node starts but never solves

Both latched inputs are required:

```bash
ros2 topic echo /hl_control/word_request
ros2 topic echo /perception/gameboard_state
```

If a new word was sent, publish a fresh board state afterwards. The word callback intentionally clears the old board state.

### Invalid word warning

The word must contain exactly five alphabetic characters. The node accepts lower-case input but converts it to upper-case.

### Model file missing

The model loader expects the path without `.zip`, but the file must exist with `.zip` appended:

```bash
ls models/wordle_ppo_latest.zip
```

Override at launch:

```bash
ros2 launch hl_control hl_control.launch.py model_path:=/absolute/path/to/wordle_ppo_v10
```

### `ModuleNotFoundError: sb3_contrib`

Install the RL dependencies in the Python environment used by ROS:

```bash
pip install stable-baselines3 sb3-contrib
```

### Empty task sequence

This can happen if the model reaches a terminal state without moves, if the board is already complete, or if no valid masked actions exist. Check the snapped cell IDs by comparing the perceived coordinates against the 0.075 m grid.

### Tasks publish but robot does not move

This is expected. `hl_control` queues tasks only. Start execution through the low-level controller:

```bash
ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once
```

### Collision objects do not appear in RViz

Check that `wordlebot_control` and MoveIt are running, and confirm messages are being published:

```bash
ros2 topic echo /wordle_bot/add_collision_object
```

The collision objects use frame `world`, so the MoveIt/RViz fixed frame must be compatible with `world`.

### Wrong tile is moved

Likely causes:

- Perception assigned the wrong letter.
- Perception assigned duplicate object IDs.
- Two tiles snapped to the same grid cell.
- The tile pose is far enough from its true cell centre that `_robot_to_cell_id()` rounded it into a neighbouring cell.

