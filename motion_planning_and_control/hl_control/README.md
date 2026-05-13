# hl_control

High-level decision-making package for the WordleBot UR3e system.

Receives a target word and the current gameboard state, runs a trained RL policy (MaskablePPO) to compute an optimal pick-and-place task sequence, and forwards the tasks to `wordlebot_control` for execution. Letter objects are also injected into the MoveIt planning scene for RViz visualisation.

The package is entirely self-contained: the RL engine source and trained model checkpoint live inside this package alongside the ROS2 node.

---

## Package Structure

```
hl_control/
├── CMakeLists.txt
├── package.xml
│
├── config/
│   └── tc2_1_board.yaml              # TC2.1 test scenario — CRANE board
│
├── launch/
│   └── hl_control.launch.py          # Launches hl_control_node
│
├── models/
│   └── wordle_ppo_latest.zip         # Trained MaskablePPO checkpoint
│
├── msg/
│   ├── LetterObject.msg              # Per-letter perception message
│   └── GameboardState.msg            # Full board state (array of LetterObject)
│
├── hl_control/                       # ROS2 Python package
│   ├── __init__.py
│   ├── hl_control_node.py            # ROS2 node — subscribes, solves, publishes tasks
│   └── rl_task_optimiser.py          # Robot↔RL coordinate bridge (inherits TaskSequencerEvaluator)
│
├── rl_task_optimiser/                # RL execution engine (no training code)
│   ├── reward.py                     # Reward function and shaping constants
│   ├── task_sequencer.py             # TaskSequencerEvaluator — loads model, runs policy
│   ├── dictionary.txt                # 5-letter word list
│   └── training_env/
│       └── wordle_env.py             # WordleSequencingEnv — Gymnasium MDP
│
└── test/
    ├── test_sim.py                   # TC2.1 simulation publisher (replaces perception + solver)
    └── demonstration_test.py         # Standalone RL demonstration (no ROS required)
```

---

## System Integration

### Full Stack

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Full Stack Overview                          │
│                                                                     │
│  [Wordle Solver]           [Perception Node]                        │
│       │ /hl_control/word_request   │ /perception/gameboard_state   │
│       │  (std_msgs/String)         │  (hl_control/GameboardState)  │
│       └──────────────┬─────────────┘                               │
│                      ▼                                              │
│              [ hl_control_node ]                                    │
│              ┌────────────────────────────┐                         │
│              │ 1. Adds letters to MoveIt  │                         │
│              │    scene (collision cubes) │                         │
│              │ 2. Snaps poses → RL grid   │                         │
│              │ 3. Runs MaskablePPO policy │                         │
│              │ 4. Converts back to robot  │                         │
│              │    space (÷10)             │                         │
│              │ 5. Publishes task sequence │                         │
│              │ 6. Logs: send start_mission│                         │
│              └────────────────────────────┘                         │
│                      │                                              │
│        /perception/letter_objects                                   │
│        (wordlebot_control/PickPlaceTask)  ← one msg per task        │
│                      │                                              │
│  [Operator / UI]  ──→  /wordle_bot/start_mission  ← arms execution  │
│                      ▼                                              │
│            [ wordlebot_control_node ]                               │
│              Plans + executes pick-and-place via MoveIt MTC         │
│                      ▼                                              │
│                [ UR3e Robot ]                                       │
└─────────────────────────────────────────────────────────────────────┘
```

`hl_control_node` is a **passive background node** — it does nothing until both a word request and a board state have arrived. It has no internal timer and does not initiate execution. After publishing tasks it logs a ready message; execution is armed by publishing `start_mission` from an external source (UI, gamification package, or operator terminal).

### TC2.1 — Isolated Test (no full system)

`test/test_sim.py` replaces both the solver and perception node. It reads `tc2_1_board.yaml` and publishes the board state and word request on startup.

```
┌─────────────────────────────────────────┐
│  python3 test/test_sim.py               │
│    reads tc2_1_board.yaml               │
│    publishes /perception/gameboard_state │
│    publishes /hl_control/word_request   │
└──────────────────┬──────────────────────┘
                   │
           [ hl_control_node ]
           (solves + logs task sequence)
                   │
   /perception/letter_objects → [ wordlebot_control_node ]
                   │
   ros2 topic pub /wordle_bot/start_mission ...  ← operator
```

---

## Class Hierarchy

```
TaskSequencerEvaluator          (rl_task_optimiser/task_sequencer.py)
        │
        │  build_env()      — create WordleEnv from perception inputs
        │  run_episode()    — run policy, return trajectory
        │  get_task_sequence() — convert trajectory to task list
        │
        ▼
RLTaskOptimiser                 (hl_control/rl_task_optimiser.py)
        │
        │  solve()          — converts robot poses → RL grid, calls above, converts back
        │  _robot_to_cell_id()
        │  _rl_to_robot()
        │  _enrich_sequence()
        │
        ▼
HLControlNode(Node, RLTaskOptimiser)   (hl_control/hl_control_node.py)
        │
        │  _word_callback()    — receives target word
        │  _board_callback()   — receives gameboard state
        │  _try_solve()        — triggers when both inputs have arrived
        │  _publish_tasks()    — sends PickPlaceTask msgs
        │  _add_letters_to_scene() — injects collision objects into MoveIt
```

---

## Coordinate System

The RL model was trained on a scaled workspace. All conversion is handled transparently inside `rl_task_optimiser.py`.

| Space | Grid spacing | X range | Y range |
|---|---|---|---|
| Robot (physical) | 0.075 m | −0.45 → +0.45 m | 0.0 → 0.45 m |
| RL model | 0.75 m | −4.5 → +4.5 m | 0.0 → 4.5 m |

**Scale factor: RL = robot × 10**

**Pick pose:** The raw perceived pose is preserved exactly (accuracy > grid snapping).

**Place pose:** The RL model's output cell centre is divided by 10 to give robot-space XY. Z is always 0.025 m, orientation is always identity.

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
- `sb3-contrib` — `pip install sb3-contrib`
- `stable-baselines3` — `pip install stable-baselines3`
- Trained model at `models/wordle_ppo_latest.zip` (included in this package)

---

## How to Run

### Step 1 — Build

From the workspace root:

```bash
colcon build --packages-select wordlebot_control hl_control
source install/setup.bash
```

> `wordlebot_control` must be built first as `hl_control` depends on its `PickPlaceTask` message. `colcon` resolves this automatically when both are listed.

---

### Step 2 — Launch the robot stack

Follow the `wordlebot_control` README to bring up the robot driver, MoveIt, and RViz. Then:

```bash
# Terminal 5
ros2 launch wordlebot_control wordle_bot_mtc.launch.py
```

---

### Step 3 — Launch hl_control

```bash
# Terminal 6
ros2 launch hl_control hl_control.launch.py
```

To use a different model checkpoint:

```bash
ros2 launch hl_control hl_control.launch.py model_path:=/path/to/wordle_ppo_v3
```

---

### Step 4 — Provide inputs

#### TC2.1 isolated test — use test_sim.py

```bash
# Terminal 7
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

The node will solve the sequence and log each step. Once satisfied:

```bash
ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once
```

#### Full system — inputs provided externally

The perception node publishes to `/perception/gameboard_state` and the solver publishes to `/hl_control/word_request`. No further action needed — `hl_control_node` solves automatically when both arrive.

---

## Topic Reference

### Subscribed by `hl_control_node`

| Topic | Type | Description |
|---|---|---|
| `/hl_control/word_request` | `std_msgs/String` | Five-letter target word (upper-case). Latched. |
| `/perception/gameboard_state` | `hl_control/GameboardState` | Array of `LetterObject` with perceived poses. Latched. |

### Published by `hl_control_node`

| Topic | Type | Description |
|---|---|---|
| `/perception/letter_objects` | `wordlebot_control/PickPlaceTask` | One message per task (pick pose, place pose, object_id). |
| `/wordle_bot/add_collision_object` | `moveit_msgs/CollisionObject` | 50 mm letter cubes added to the MoveIt scene for RViz. |

### Sent externally to arm execution

| Topic | Type | Publisher |
|---|---|---|
| `/wordle_bot/start_mission` | `std_msgs/Bool` | UI / gamification package (full system) or operator terminal (test) |

---

## Message Definitions

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

**`wordlebot_control/PickPlaceTask`**
```
geometry_msgs/PoseStamped pick_pose
geometry_msgs/Pose place_pose
string object_id
```

---

## TC2.1 Board YAML Format

Edit `config/tc2_1_board.yaml` to change the test scenario.

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

`object_id` follows the format `{LETTER}_object_{n}` and must be unique per letter. It is used as the MoveIt collision object ID throughout planning and execution.

---

## Manual Terminal Control

Publish a board state and word request manually without `test_sim.py`:

```bash
ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'A', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.308, y: 0.302, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.377, y: 0.154, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'C', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.442, y: 0.073, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'E', object_id: 'N_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.153, y: 0.457, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'N', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.377, y: 0.224, z: 0.025},
                   orientation: {w: 1.0}}}}
  ]}"

ros2 topic pub --once /hl_control/word_request std_msgs/msg/String "{data: 'CRANE'}"
```

Monitor the published task sequence:

```bash
ros2 topic echo /perception/letter_objects
```

---

## Standalone RL Demonstration (no ROS)

`test/demonstration_test.py` runs the full RL evaluation loop without any ROS infrastructure. It loads the model, runs the policy against all four curriculum scenarios, and saves comparison figures and animated GIFs to `rl_task_optimiser/logs/`.

```bash
python3 test/demonstration_test.py
python3 test/demonstration_test.py --no-animate   # skip GIF generation
```

---

## Future Integration Points

| Concern | How to wire it |
|---|---|
| Real perception | Replace `test_sim.py` with a perception node publishing `GameboardState` to `/perception/gameboard_state` |
| Wordle solver | Publish guesses to `/hl_control/word_request`; subscribe to `/wordle_bot/mission_complete` to trigger the next guess |
| TC2.2 (wrong letters in Wordle slots) | Same YAML format — add entries with XY in the Wordle zone. `RLTaskOptimiser` always uses stage=3 which handles clear-and-replace automatically |
| Different model checkpoint | Set `model_path` in the launch argument or pass `None` to use `models/wordle_ppo_latest.zip` |
