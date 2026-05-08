# hl_control

High-level decision-making package for the WordleBot UR3e system.

Receives a target word and the current gameboard state, runs a trained RL policy (MaskablePPO) to sequence pick-and-place tasks, and forwards them to `wordlebot_control` for execution. Also visualises the letter objects in RViz by injecting them into the MoveIt planning scene.

---

## System Integration

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Full Stack Overview                          │
│                                                                     │
│  [Wordle Solver]          [Perception Node]                         │
│       │ /hl_control/word_request   │ /perception/gameboard_state    │
│       │  (std_msgs/String)         │  (hl_control/GameboardState)   │
│       └──────────────┬────────────┘                                 │
│                      ▼                                              │
│              [ hl_control_node ]                                    │
│              ┌───────────────────────────┐                          │
│              │ 1. Adds letters to RViz   │                          │
│              │    (collision objects)    │                          │
│              │ 2. Snaps poses → RL grid  │                          │
│              │ 3. Runs RL model          │                          │
│              │ 4. Converts back to robot │                          │
│              │    space (÷10)            │                          │
│              │ 5. Publishes task queue   │                          │
│              └───────────────────────────┘                          │
│                      │                                              │
│        /perception/letter_objects                                   │
│        (wordlebot_control/PickPlaceTask)  ← one msg per task        │
│        /wordle_bot/start_mission          ← arms execution          │
│                      ▼                                              │
│            [ wordlebot_control_node ]                               │
│              Plans + executes pick-and-place via MoveIt MTC         │
│                      ▼                                              │
│                [ UR3e Robot ]                                       │
└─────────────────────────────────────────────────────────────────────┘
```

### For TC2.1 (simulation / testing without real perception or solver)

The `test_env_publisher` node replaces both the solver and the perception node. It reads a YAML file defining letter positions and publishes the board state and target word automatically on startup.

```
┌──────────────────────────────────────────────┐
│  [test_env_publisher]                        │
│    reads tc2_1_board.yaml                    │
│    publishes gameboard_state (latched)       │
│    publishes word_request                    │
└──────────────────────┬───────────────────────┘
                       │
               [ hl_control_node ]
                       │
           [ wordlebot_control_node ]
```

---

## Coordinate System

The RL model was trained on a scaled workspace. All coordinate conversion is handled transparently inside `rl_task_optimiser.py`.

| Space | Grid spacing | X range | Y range |
|---|---|---|---|
| Robot (physical) | 0.075 m | −0.45 → +0.45 m | 0.0 → 0.45 m |
| RL model | 0.75 m | −4.5 → +4.5 m | 0.0 → 4.5 m |

**Scale factor: RL = robot × 10**

**Pick pose:** The raw perceived pose is preserved exactly and passed to the robot unchanged (accuracy > grid alignment).

**Place pose:** The RL model's output cell centre is divided by 10 to give robot-space XY. Z is always 0.025 m and orientation is always identity (w=1).

Example:
```
Perceived pick:  (0.22, 0.31, 0.025, w=1)   ← raw from perception
Scaled to RL:    (2.20, 3.10)
Snapped to grid: (2.25, 3.00)               → cell_id for model
Model output place cell → RL centre: (−1.50, 2.25)
Robot place pose:        (−0.15, 0.225, 0.025, w=1)
Robot pick returned:     (0.22, 0.31, 0.025, w=1)   ← original preserved
```

**Wordle slots (robot space):**

| Slot | X (m) | Y (m) |
|---|---|---|
| 0 | −0.150 | 0.225 |
| 1 | −0.075 | 0.225 |
| 2 |  0.000 | 0.225 |
| 3 |  0.075 | 0.225 |
| 4 |  0.150 | 0.225 |

---

## Prerequisites

Everything from `wordlebot_control` (ROS 2 Humble+, MoveIt 2, UR driver) plus:

- Python 3.10+
- `sb3-contrib` (`pip install sb3-contrib`)
- `stable-baselines3` (`pip install stable-baselines3`)
- Trained model at `rl_task_optimiser/models/wordle_ppo_latest.zip`
- `rl_task_optimiser` package at `/home/connorlindsell/git/AiRobotics/rl_task_optimiser`

---

## How to Run

### Step 1 — Build both packages

From the workspace root (`/home/connorlindsell/git/RS2/motion_planning_and_control`):

```bash
colcon build --packages-select wordlebot_control hl_control
source install/setup.bash
```

> `wordlebot_control` must be built first because `hl_control` depends on its `PickPlaceTask` message type. `colcon` resolves this automatically when both are listed together.

---

### Step 2 — Launch the robot stack

Follow the `wordlebot_control` README to bring up the robot driver, MoveIt, and RViz (Terminals 1–4). Then launch the control node:

```bash
# Terminal 5
ros2 launch wordlebot_control wordle_bot_mtc.launch.py
```

---

### Step 3 — Launch hl_control

#### TC2.1 — Full pipeline with test publisher (simulation)

Launches `hl_control_node` and `test_env_publisher` together. The publisher reads `tc2_1_board.yaml` and fires after 2 seconds to let all nodes connect.

```bash
# Terminal 6
ros2 launch hl_control hl_control.launch.py use_test_publisher:=true word:=CRANE
```

To test a different word (letters must exist on the board):

```bash
ros2 launch hl_control hl_control.launch.py use_test_publisher:=true word:=CRANE \
  config_path:=/path/to/your_board.yaml
```

To point at a different model checkpoint:

```bash
ros2 launch hl_control hl_control.launch.py use_test_publisher:=true word:=CRANE \
  model_path:=/home/connorlindsell/git/AiRobotics/rl_task_optimiser/models/wordle_ppo_v3
```

#### Standalone — Real perception / solver integration

```bash
# Terminal 6
ros2 launch hl_control hl_control.launch.py
```

Then publish the board state and word separately (see Manual Control below).

---

## Topic Reference

### Subscribed by `hl_control_node`

| Topic | Type | Description |
|---|---|---|
| `/hl_control/word_request` | `std_msgs/String` | Five-letter target word (upper-case). Solving triggers when both word and board state have been received. |
| `/perception/gameboard_state` | `hl_control/GameboardState` | Array of `LetterObject` (letter + PoseStamped + object_id). Latched — can be published once at startup. |

### Published by `hl_control_node`

| Topic | Type | Description |
|---|---|---|
| `/perception/letter_objects` | `wordlebot_control/PickPlaceTask` | One message per task, published sequentially. Each carries pick_pose, place_pose, and object_id. |
| `/wordle_bot/add_collision_object` | `moveit_msgs/CollisionObject` | Adds 50 mm letter cubes to the MoveIt scene for RViz visualisation. Published once per letter on board-state receipt. |
| `/wordle_bot/start_mission` | `std_msgs/Bool` | Published `true` after all tasks are queued to arm execution in `wordlebot_control`. |

### Message Definitions

**`hl_control/LetterObject`**
```
string letter
geometry_msgs/PoseStamped pose
string object_id
```

**`hl_control/GameboardState`**
```
hl_control/LetterObject[] letters
```

**`wordlebot_control/PickPlaceTask`** (updated)
```
geometry_msgs/PoseStamped pick_pose
geometry_msgs/Pose place_pose
string object_id
```

---

## Manual Terminal Control

Use these to test the hl_control interface without the test publisher.

### Publish a board state (latched)

```bash
ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.448, y: 0.072, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.377, y: 0.078, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.302, y: 0.073, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'N', object_id: 'N_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.373, y: 0.077, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.447, y: 0.074, z: 0.025},
                   orientation: {w: 1.0}}}}
  ]}"
```

### Send a word request

```bash
ros2 topic pub --once /hl_control/word_request std_msgs/msg/String "{data: 'CRANE'}"
```

### Monitor the task sequence as it is published

```bash
ros2 topic echo /perception/letter_objects
```

### Monitor mission state

```bash
ros2 topic echo /wordle_bot/robot_state
ros2 topic echo /wordle_bot/mission_complete
```

---

## TC2.1 Board YAML Format

Edit `config/tc2_1_board.yaml` to change the test scenario. Each entry is one letter block with its raw perceived pose in robot space.

```yaml
word: CRANE

letters:
  - letter: C
    object_id: C_object_1
    x: -0.448     # robot-space X (metres)
    y:  0.072     # robot-space Y (metres)
    z:  0.025     # height — same for all letters
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0       # identity orientation
```

The `object_id` format is `{LETTER}_object_{n}` (1-indexed per letter type). It is used as the MoveIt collision object ID throughout planning and execution, so it must be unique per letter on the board.

---

## Package Structure

```
hl_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── tc2_1_board.yaml          # TC2.1 test scenario (CRANE)
├── launch/
│   └── hl_control.launch.py      # Main launch file
├── msg/
│   ├── LetterObject.msg           # Per-letter perception message
│   └── GameboardState.msg         # Full board state (array of LetterObject)
└── hl_control/
    ├── __init__.py
    ├── hl_control_node.py         # ROS2 node — orchestration
    ├── rl_task_optimiser.py       # RL logic — wraps TaskSequencerEvaluator
    └── test_env_publisher.py      # TC2.1 test publisher
```

---

## Integration with `rl_task_optimiser`

`rl_task_optimiser.py` subclasses `TaskSequencerEvaluator` from `test.py` in the rl_task_optimiser repo. The three inherited methods used at runtime are:

| Method | Role |
|---|---|
| `build_env(stage, word, fixed_positions)` | Creates a `WordleEnv` with the perceived board state |
| `run_episode(env)` | Runs the MaskablePPO policy to completion, returns a trajectory |
| `get_task_sequence(trajectory)` | Converts the trajectory into a list of pick/place cell pairs |

The `RLTaskOptimiser.solve()` method wraps all three steps and handles coordinate conversion in and out of RL space. It always uses `stage=3` (C3 masking), which handles both C2-like boards (empty Wordle slots, TC2.1) and C3-like boards (some wrong letters already in slots, TC2.2+).

The PYTHONPATH is set by the launch file to point at the `rl_task_optimiser` directory — no files are copied or installed from that repo.

---

## Future Integration Points

| Concern | How to wire it |
|---|---|
| Real perception | Replace `test_env_publisher` with a perception node that publishes `GameboardState` to `/perception/gameboard_state` |
| Wordle solver | Publish the next guess to `/hl_control/word_request`; subscribe to `/wordle_bot/mission_complete` to know when to send the next word |
| TC2.2 (letters in Wordle slots) | Same YAML format — add entries with XY coordinates in the Wordle zone. `rl_task_optimiser` uses stage=3 which handles clearing and re-placing |
| Phase 2 model (loose masking) | Change `model_path` launch argument to point at `wordle_ppo_loose_latest` |
