# hl_control — Technical Documentation

## Project Overview

`hl_control` is the high-level decision-making subsystem of the WordleBot robotic system. It receives a target five-letter word and the current gameboard state from the perception subsystem, runs a trained reinforcement-learning (RL) policy (MaskablePPO) to compute an optimal pick-and-place task sequence, and publishes that sequence to `wordlebot_control` for physical execution by a UR3e robot arm.

---

## Key Features

- **RL-based task sequencing** — a MaskablePPO policy determines the order and destination of each letter tile pick-and-place action to minimise total travel distance.
- **Curriculum-trained policy** — trained across four progressive difficulty stages (C1–C4): no distractors → distractors → blocked Wordle slots → full autonomy.
- **Coordinate bridge** — robot-space poses from perception (0.075 m grid) are converted to RL training space (0.75 m grid, ×10 scale) and back, transparently.
- **MoveIt scene injection** — each detected letter tile is registered as a 50 mm collision cube in the MoveIt planning scene for RViz visualisation and collision avoidance.
- **Passive architecture** — the node does nothing until both a word request and a board state have arrived; it has no timer and does not initiate execution.
- **Standalone testability** — a simulation publisher (`test_sim.py`) and a no-ROS demonstration script (`demonstration_test.py`) allow isolated testing without the full robot stack.

---

## Dependencies

### Hardware

| Component | Details |
|---|---|
| Robot arm | Universal Robots UR3e |
| Gripper | OnRobot parallel jaw gripper |
| Letter tiles | 50 mm 3D-printed cubes, letter printed/engraved on top face |
| Gameboard | 0.9 m (X) × 0.45 m (Y) flat surface divided into 0.075 × 0.075 m grid cells |

> **Figure:** *(Insert photograph of the physical environment here)*

**Workspace layout:**
- Grid: 13 columns × 7 rows (91 cells total), cell spacing 0.075 m
- Robot origin sits at grid column 6, row 0 → world coordinate (0.0, 0.0)
- X range: −0.45 m to +0.45 m
- Y range: 0.0 m to +0.45 m
- **Wordle zone:** 5 cells at Y = 0.225 m (row 3), columns 4–8, X ∈ {−0.150, −0.075, 0.000, +0.075, +0.150} m

### Software

| Requirement | Version / Notes |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10+ |
| MoveIt 2 | Humble |
| UR ROS 2 Driver | Compatible with Humble |
| `stable-baselines3` | `pip install stable-baselines3` |
| `sb3-contrib` | `pip install sb3-contrib` (provides `MaskablePPO`) |
| `gymnasium` | Installed as a dependency of `stable-baselines3` |
| `numpy`, `matplotlib` | Standard scientific Python stack |
| `PyYAML` | For reading test board config files |

---

## Installation

### Hardware Setup

1. Mount the UR3e arm at the origin of the gameboard coordinate frame. The robot base should be aligned so that the end-effector reaches the full 0.9 × 0.45 m gameboard surface.
2. Attach the OnRobot parallel jaw gripper. Ensure finger spacing is calibrated to grip 50 mm letter cubes reliably.
3. Position the gameboard flat in front of the robot. The Wordle zone (five destination slots) should sit at Y ≈ 0.225 m from the robot base, centred on X = 0.
4. Place letter tile cubes in the staging area (outer grid cells, away from the Wordle zone). Tile heights should be consistent so that Z ≈ 0.025 m above the board surface.

### Software Installation

**Step 1 — Clone the repository**

```bash
git clone <repository-url> ~/rs2_ws/src
cd ~/rs2_ws
```

**Step 2 — Install ROS 2 dependencies**

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Step 3 — Install Python RL dependencies**

```bash
pip install stable-baselines3 sb3-contrib
```

**Step 4 — Build the workspace**

`wordlebot_control` must be built before `hl_control` because `hl_control` depends on its `PickPlaceTask` message. `colcon` resolves this automatically when both packages are listed:

```bash
colcon build --packages-select wordlebot_control hl_control
```

**Step 5 — Source the workspace**

```bash
source install/setup.bash
```

Add to `~/.bashrc` to avoid re-sourcing every terminal:

```bash
echo "source ~/rs2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the System

### Full System (with robot, perception, and solver)

The following assumes the UR3e driver, MoveIt, RViz, and `wordlebot_control` are already running. Refer to the `wordlebot_control` documentation for those steps.

**Terminal 1 — Launch hl_control**

```bash
ros2 launch hl_control hl_control.launch.py
```

Expected output:
```
[hl_control_node]: HLControlNode ready — waiting for word request and board state.
```

The node now waits silently until both inputs arrive. No further action is needed — the perception node and Wordle solver publish to the required topics automatically.

**Once hl_control has solved and published the task sequence**, it logs each step and prints:
```
[hl_control_node]: All tasks queued. To start execution, run:
  ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once
```

**Trigger execution:**

```bash
ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once
```

The robot will then execute each pick-and-place step in sequence via `wordlebot_control`.

---

### TC2.1 Isolated Test (no perception or solver required)

This is the recommended way to verify `hl_control` independently.

**Terminal 1 — Launch hl_control**

```bash
ros2 launch hl_control hl_control.launch.py
```

**Terminal 2 — Run the simulation publisher**

```bash
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

`test_sim.py` reads the YAML board configuration and publishes once to `/perception/gameboard_state` and `/hl_control/word_request`. After ~2 seconds, `hl_control_node` will log the full task sequence.

**Expected terminal output (Terminal 1):**

```
[hl_control_node]: HLControlNode ready — waiting for word request and board state.
[hl_control_node]: Board state received: 5 letter(s).
[hl_control_node]: Word request received: CRANE
[hl_control_node]: Solving: word="CRANE" with 5 letter(s) on board.
[hl_control_node]: Published 5 collision object(s) to MoveIt scene.
[hl_control_node]: Task sequence (5 step(s)):
[hl_control_node]:   Step 1: Pick 'C' from cell 13 and place in Wordle slot 0 | ...
[hl_control_node]:   Step 2: Pick 'R' from cell 14 and place in Wordle slot 1 | ...
...
[hl_control_node]: All tasks queued. To start execution, run:
  ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once
```

**Trigger execution (if robot stack is running):**

```bash
ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once
```

---

### Standalone RL Demonstration (no ROS required)

Runs the RL policy against all four curriculum scenarios and saves comparison figures and animated GIFs to `rl_task_optimiser/logs/`. No robot or ROS installation needed.

```bash
python3 test/demonstration_test.py
```

To skip GIF generation (faster):

```bash
python3 test/demonstration_test.py --no-animate
```

---

## Subsystem Specifics

### Purpose

`hl_control` sits between the perception and motion execution layers. Its sole responsibility is to decide **which letter goes where and in what order**, given:
- The five-letter word to spell (from the Wordle solver)
- The current physical positions and identities of letter tiles (from the perception stack)

It does not control the robot directly. It outputs a task sequence that `wordlebot_control` executes.

---

### Architecture

```
TaskSequencerEvaluator          (rl_task_optimiser/task_sequencer.py)
        |
        |  build_env()         — create WordleEnv from perception inputs
        |  run_episode()       — run policy, return trajectory
        |  get_task_sequence() — convert trajectory to ordered task list
        |
        v
RLTaskOptimiser                 (hl_control/rl_task_optimiser.py)
        |
        |  solve()             — converts robot poses to RL grid, calls above, converts back
        |  _robot_to_cell_id() — snaps robot-space (x,y) to nearest grid cell_id
        |  _rl_to_robot()      — converts RL cell centre back to robot space
        |  _enrich_sequence()  — attaches full pick/place poses and object_ids to each step
        |
        v
HLControlNode(Node, RLTaskOptimiser)   (hl_control/hl_control_node.py)
        |
        |  _word_callback()        — receives target word from /hl_control/word_request
        |  _board_callback()       — receives gameboard state from /perception/gameboard_state
        |  _try_solve()            — fires when both inputs have arrived
        |  _add_letters_to_scene() — injects MoveIt collision objects
        |  _publish_tasks()        — sends PickPlaceTask messages
```

---

### Key Files

| File | Role |
|---|---|
| `hl_control/hl_control_node.py` | ROS 2 node: subscribes, triggers solve, publishes tasks and collision objects |
| `hl_control/rl_task_optimiser.py` | Coordinate bridge between robot space and RL training space |
| `rl_task_optimiser/task_sequencer.py` | Loads the MaskablePPO model and runs the policy |
| `rl_task_optimiser/training_env/wordle_env.py` | Gymnasium MDP: grid state, action masking, observation builder |
| `rl_task_optimiser/reward.py` | Reward function and shaping constants |
| `rl_task_optimiser/dictionary.txt` | 5-letter word list used during RL training |
| `models/wordle_ppo_latest.zip` | Pre-trained MaskablePPO checkpoint |
| `config/tc2_1_board.yaml` | TC2.1 test board: CRANE, 5 letters in outer staging |
| `launch/hl_control.launch.py` | ROS 2 launch file |
| `test/test_sim.py` | Simulation publisher — replaces perception and solver for isolated tests |
| `test/demonstration_test.py` | Standalone RL evaluation — no ROS required |
| `msg/LetterObject.msg` | Per-letter perception message |
| `msg/GameboardState.msg` | Full board message (array of LetterObject) |

---

### Topics

#### Subscribed by `hl_control_node`

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/hl_control/word_request` | `std_msgs/String` | Latched (TRANSIENT_LOCAL, depth 1) | Five-letter target word in upper-case (e.g. `"CRANE"`). Invalid if not exactly 5 alphabetic characters. |
| `/perception/gameboard_state` | `hl_control/GameboardState` | Latched (TRANSIENT_LOCAL, depth 1) | Array of `LetterObject`: letter identity, object ID, and 6-DOF pose in the `world` frame. |

#### Published by `hl_control_node`

| Topic | Type | Description |
|---|---|---|
| `/perception/letter_objects` | `wordlebot_control/PickPlaceTask` | One message per task step. Contains pick pose (stamped, `world` frame), place pose, and `object_id`. |
| `/wordle_bot/add_collision_object` | `moveit_msgs/CollisionObject` | One 50 mm BOX collision object per detected letter, added to the MoveIt planning scene. Frame: `world`. |

#### External — arms execution

| Topic | Type | Who publishes |
|---|---|---|
| `/wordle_bot/start_mission` | `std_msgs/Bool` | UI / gamification package (full system), or operator terminal during testing |

---

### Message Definitions

**`hl_control/LetterObject`**
```
string letter                        # single upper-case letter, e.g. "C"
geometry_msgs/PoseStamped pose       # 6-DOF pose in world frame
string object_id                     # unique ID, e.g. "C_object_1"
```

**`hl_control/GameboardState`**
```
hl_control/LetterObject[] letters    # all detected letter tiles
```

**`wordlebot_control/PickPlaceTask`** (defined in `wordlebot_control`)
```
geometry_msgs/PoseStamped pick_pose  # pick location, world frame, stamped
geometry_msgs/Pose place_pose        # place location, world frame (no stamp)
string object_id                     # matches collision object ID in MoveIt scene
```

---

### Coordinate System

The RL model was trained on a scaled workspace. All conversion is handled internally by `rl_task_optimiser.py`.

| Space | Grid spacing | X range | Y range |
|---|---|---|---|
| Robot (physical) | 0.075 m | −0.45 m → +0.45 m | 0.0 m → +0.45 m |
| RL model | 0.75 m | −4.5 m → +4.5 m | 0.0 m → +4.5 m |

**Scale factor: RL = robot × 10**

- **Pick pose:** the raw perceived pose from the perception system is preserved exactly (perception accuracy > grid-snapped accuracy).
- **Place pose:** the RL model outputs a destination grid cell; its centre is divided by 10 to give robot-space XY. Z is always 0.025 m, orientation is always identity quaternion (0, 0, 0, 1).

**Wordle slot positions (robot space):**

| Slot index | X (m) | Y (m) |
|---|---|---|
| 0 (letter 1) | −0.150 | 0.225 |
| 1 (letter 2) | −0.075 | 0.225 |
| 2 (letter 3) |  0.000 | 0.225 |
| 3 (letter 4) | +0.075 | 0.225 |
| 4 (letter 5) | +0.150 | 0.225 |

---

### RL Environment Summary

The `WordleSequencingEnv` (Gymnasium) models the task as a symbolic pick-and-place problem on a 13 × 7 grid.

| Property | Value |
|---|---|
| Action space | `Discrete(8281)` — `source_cell_id × 91 + dest_cell_id` |
| Observation space | `Box(2686,)` floats — robot position (2), cell states (91 × 28), needs-clearing flags (5), target word encoding (130), stage indicator (1) |
| Max steps per stage | C1: 10, C2: 15, C3: 25, C4: 35 |
| Holdout test word | `GREAT` (excluded from training word list) |

**Curriculum stages:**

| Stage | Board configuration | Action mask |
|---|---|---|
| C1 | 5 correct letters in random outer staging, no distractors | Tight — only correct letter into correct empty Wordle slot |
| C2 | 5 correct + 5 distractor letters in random outer staging | Tight — same as C1, distractors are never valid moves |
| C3 | All 5 Wordle slots blocked by wrong letters; 5 correct + 10 distractors in outer staging | Semi-constrained — clearing from Wordle to outer staging allowed; placing into Wordle only if letter matches |
| C4 | Identical board to C3 | Loose — any move to any empty non-forbidden cell |

In deployment, the node always uses **stage 3**, which handles both C2-like boards (empty Wordle slots) and C3-like boards (blocked Wordle slots requiring clearing).

**Reward function summary:**

| Event | Reward |
|---|---|
| Word complete (all 5 slots filled correctly) | +100 |
| Correct letter placed in correct slot (once per slot) | +20 |
| Wrong-letter tile cleared from Wordle slot to staging | +15 |
| Placing a letter in the wrong Wordle slot | −20 |
| Evicting a correctly-placed letter from its slot | −10 |
| Step penalty (per action) | −1 |
| Travel cost | −2 × distance (RL metres) |

---

### Configurable Parameters

#### Launch argument

| Parameter | Default | Description |
|---|---|---|
| `model_path` | `models/wordle_ppo_latest` (installed package) | Absolute path to the MaskablePPO checkpoint, **without** the `.zip` extension. |

Example — use a specific checkpoint:

```bash
ros2 launch hl_control hl_control.launch.py model_path:=/path/to/wordle_ppo_v10
```

#### Constants in source (not exposed as ROS parameters)

| Constant | File | Value | Description |
|---|---|---|---|
| `LETTER_CUBE_SIZE` | `hl_control_node.py` | `0.05` m | Side length of the MoveIt collision cube for each letter tile |
| `SOLVE_STAGE` | `hl_control_node.py` | `3` | Curriculum stage passed to the RL environment at inference |
| `ROBOT_SCALE` | `rl_task_optimiser.py` | `10.0` | Multiplier from robot space to RL training space |
| `PLACE_Z` | `rl_task_optimiser.py` | `0.025` m | Fixed Z for all place poses |
| `GRID_STEP_RL` | `rl_task_optimiser.py` | `0.75` m | RL grid cell size (used for cell-ID snapping) |

---

### How to Test Independently

**Option 1 — ROS simulation test (TC2.1 board, CRANE)**

Run in two terminals from the `hl_control` package directory:

```bash
# Terminal 1
ros2 launch hl_control hl_control.launch.py

# Terminal 2
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

Verify the task sequence is logged in Terminal 1. No robot is needed for this test.

**Option 2 — Custom board via YAML**

Edit or create a YAML file following the format of `config/tc2_1_board.yaml`, then pass it to `test_sim.py`:

```bash
python3 test/test_sim.py --ros-args \
  -p config_path:=/absolute/path/to/my_board.yaml
```

Board YAML format:

```yaml
word: CRANE

letters:
  - letter: C
    object_id: C_object_1
    x: -0.448     # robot-space X (metres)
    y:  0.072     # robot-space Y (metres)
    z:  0.025     # height above board surface
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0       # identity orientation
```

Rules for the YAML:
- `word` must be exactly 5 alphabetic characters.
- `object_id` must be unique per letter; convention is `{LETTER}_object_{n}`.
- `x`, `y`, `z` are in metres, in the `world` frame.
- Orientation quaternion (qx, qy, qz, qw) should be identity `(0, 0, 0, 1)` unless tiles are physically tilted.

**Option 3 — Manual topic publish (no YAML)**

```bash
ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.448, y: 0.072, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.377, y: 0.154, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.302, y: 0.302, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'N', object_id: 'N_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.373, y: 0.220, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.157, y: 0.304, z: 0.025},
                   orientation: {w: 1.0}}}}
  ]}"

ros2 topic pub --once /hl_control/word_request std_msgs/msg/String "{data: 'CRANE'}"
```

Monitor published tasks:

```bash
ros2 topic echo /perception/letter_objects
```

**Option 4 — Standalone RL demo (no ROS)**

```bash
python3 test/demonstration_test.py
```

Saves per-scenario comparison figures and animated GIFs to `rl_task_optimiser/logs/`. Useful for verifying the model loads and produces valid sequences entirely outside ROS.

---

### Known Limitations and Assumptions

- **Stage fixed to 3:** the node always calls the RL environment at stage 3 regardless of the actual board configuration. Stage 3 handles both empty-slot and blocked-slot scenarios. Stages 1, 2, and 4 are only used during training and evaluation.
- **One solve per input pair:** once a word request and board state have been received, `_pending_word` is cleared. A new solve requires new messages on both topics. If the board changes mid-solve this is not handled.
- **Place orientation is always identity:** the RL model outputs only XY placement; the node always sets place pose orientation to (0, 0, 0, 1). Tiles must be placed upright by the gripper regardless of how they were picked.
- **Place Z is fixed at 0.025 m:** the node hard-codes Z for all place poses. The physical board surface height relative to the robot base must match this value.
- **Perception noise tolerance:** pick poses are used exactly as received from perception. If perceived positions are far from the physical tile centres, gripper misses can occur. The robot-to-cell-ID snapping in `rl_task_optimiser.py` uses rounding to the nearest 0.075 m grid cell.
- **No replanning:** if a physical pick-and-place fails mid-sequence (e.g. gripper drop), `hl_control` is not notified and does not replan. Recovery must be handled externally.
- **Five-letter words only:** word requests with any other length are rejected with a warning.
- **Model must exist:** the node will fail to start if `models/wordle_ppo_latest.zip` is missing.

---

## Troubleshooting & FAQs

**The node starts but never solves — logs say "waiting for word request and board state."**

Both `/hl_control/word_request` and `/perception/gameboard_state` must be received before solving triggers. Check that both publishers are running:

```bash
ros2 topic list | grep -E "word_request|gameboard_state"
ros2 topic echo /hl_control/word_request
ros2 topic echo /perception/gameboard_state
```

If using `test_sim.py`, confirm the `config_path` parameter points to an existing file:

```bash
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

---

**Error: `No model at /path/to/wordle_ppo_latest.zip — train first.`**

The model checkpoint is missing. Verify it is present:

```bash
ls models/wordle_ppo_latest.zip
```

If missing, the file should be included in the repository under `models/`. If you want to use a different checkpoint:

```bash
ros2 launch hl_control hl_control.launch.py model_path:=/absolute/path/to/checkpoint
```

Note: do **not** include the `.zip` extension in the path.

---

**Error: `ModuleNotFoundError: No module named 'sb3_contrib'`**

Install the required Python packages:

```bash
pip install stable-baselines3 sb3-contrib
```

---

**Word request is received but rejected — "Received invalid word"**

The word must be exactly 5 upper-case alphabetic characters. The node normalises to upper-case automatically, but rejects words that are not 5 letters or contain non-alphabetic characters.

---

**RL solver returned an empty task sequence.**

This should not occur for valid boards but can happen if all letters are already correctly placed (no moves needed) or if the RL model cannot find valid actions under the action mask. Check the board YAML — confirm the letters are not already in the Wordle slots at the correct positions.

---

**Collision objects do not appear in RViz.**

Ensure the MoveIt planning scene is running and subscribing to `/wordle_bot/add_collision_object`. Verify the `wordlebot_control` launch has completed before running `hl_control`. The collision objects use frame `world` — confirm the `world` frame is present in the TF tree:

```bash
ros2 run tf2_ros tf2_echo world base_link
```

---

**Task sequence is published but robot does not move.**

The robot does not move until `start_mission` is published. After the task sequence is logged:

```bash
ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once
```

---

**How do I change which model checkpoint is used without rebuilding?**

Pass the `model_path` launch argument at runtime:

```bash
ros2 launch hl_control hl_control.launch.py model_path:=/absolute/path/to/wordle_ppo_v10
```

The path must be absolute and must not include the `.zip` extension.

---

**How do I add a new test scenario (different word and board layout)?**

1. Copy `config/tc2_1_board.yaml` to a new file (e.g. `config/my_test.yaml`).
2. Edit the `word` field and the `letters` array with the new letter positions.
3. Pass the new file to `test_sim.py`:

```bash
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/my_test.yaml
```

The `word` field in the YAML is used by default; pass `-p word:=XXXXX` to override it at runtime.
