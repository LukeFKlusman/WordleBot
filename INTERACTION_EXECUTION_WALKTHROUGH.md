# Interaction Execution Technical Walkthrough

## High-Level Overview

The **Interaction Execution** subsystem is the orchestrator of the RS2 WordleBot system. It's responsible for:
- Managing the mission lifecycle (IDLE → SCANNING → READY_TO_MOVE → MOVING → IDLE)
- Coordinating safety checks (halting motion if humans are detected)
- Running a Qt-based GUI for operator control and visualization
- Publishing commands to other subsystems (Perception, Motion Planning, Gamification)
- Subscribing to sensor feedback and game state

**Package Location:** `/home/elijahs04/ros2_ws/src/RS2/interaction_execution/`

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────┐
│              INTERACTION EXECUTION SUBSYSTEM                 │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │  MissionCoordinator (C++ ROS2 Node)               │    │
│  │  - Behavior tree executor                         │    │
│  │  - Mission state machine (IDLE/SCANNING/etc)      │    │
│  │  - Safety guard logic                             │    │
│  │  - Perception dispatch & tracking                 │    │
│  │  - Motion coordination                            │    │
│  └────────────────────────────────────────────────────┘    │
│                         ▲                                   │
│                         │                                   │
│  ┌──────────────────────┴──────────────────────┐           │
│  │  Qt GUI (interaction_execution_node)       │           │
│  │  ┌──────────────────────────────────────┐  │           │
│  │  │ MainWindow (central controller)      │  │           │
│  │  │ - CameraView (RealSense feed)        │  │           │
│  │  │ - RvizSimView (RViz simulation)      │  │           │
│  │  │ - RvizMoveItView (MoveIt planning)   │  │           │
│  │  │ - WordleView (game display)          │  │           │
│  │  │ - Diagnostics window                 │  │           │
│  │  │ - Voice control integration          │  │           │
│  │  └──────────────────────────────────────┘  │           │
│  └──────────────────────────────────────────────┘           │
└──────────────────────────────────────────────────────────────┘
```

---

## Core Components

### 1. **MissionCoordinator** (The Behavior Tree Executor)

**File:** [interaction_execution/src/mission_coordinator.cpp](interaction_execution/src/mission_coordinator.cpp)
**Header:** [interaction_execution/include/interaction_execution/mission_coordinator.hpp](interaction_execution/include/interaction_execution/mission_coordinator.hpp)

#### Purpose
Implements a behavior tree (not a formal library, but tree-like control flow) that orchestrates the entire mission lifecycle. It runs every 500ms via a heartbeat timer and decides what to do based on:
- Current mission state
- Human detection status
- Perception feedback (scan completion, detections)
- Motion feedback (movement completion)
- Operator commands (START, STOP, HOME)

#### Key States (enum MissionState)
```cpp
IDLE              // Waiting for operator START
SCANNING          // Perception is scanning the board
READY_TO_MOVE     // Perception done, gamification computed move, ready to pick letters
MOVING            // Robot arm is executing the move
STOPPED           // Human detected, motion halted
HOMING            // Returning arm to safe home position
ERROR             // Something went wrong
```

#### Behavior Tree Structure
The mission coordinator has a tree-like decision structure with these branches:

**SafetyGuard Branch:**
- Monitors `/perception/human_detected`
- If human detected: publish `/wordle_bot/stop_mission` and transition to STOPPED
- If human leaves: publish `/wordle_bot/resume_mission` and transition back

**CommandBranch:**
- Monitors `/wordle_bot/mission_cmd` for operator commands: START, STOP, HOME
- Normalizes commands to uppercase
- Handles user input transitions

**ScanBranch:**
- Triggers when state is SCANNING
- Publishes `/mission/state` = "SCANNING" to wake up Perception
- Waits for `/perception/status` and `/perception/detections`
- Counts detected letters; transitions to READY_TO_MOVE when enough detections

**MotionBranch:**
- Triggers when state is READY_TO_MOVE or MOVING
- Monitors `/gamification/mission_state` (JSON: word + pick poses)
- Converts mission_state to `PoseArray` and publishes to `/wordle_bot/set_mission`
- Publishes `/wordle_bot/start_mission` = true
- Waits for `/wordle_bot/motion_complete` before transitioning to IDLE

#### Key Methods

| Method | Purpose |
|--------|---------|
| `tickTree()` | Called every 500ms; executes the full behavior tree |
| `tickSafetyGuard()` | Checks human detection, handles emergency stop/resume |
| `tickCommandBranch()` | Processes operator commands |
| `tickScanBranch()` | Triggers perception scans, counts detections |
| `tickMotionBranch()` | Converts gamification output to motion commands |
| `handleMissionCommand()` | ROS2 callback for `/wordle_bot/mission_cmd` |
| `handleHumanDetected()` | ROS2 callback for `/perception/human_detected` |
| `handlePerceptionStatus()` | Tracks scan progress |
| `handlePerceptionDetections()` | Receives letter detections as JSON |
| `handleMotionComplete()` | Called when robot finishes a move |
| `transitionTo()` | Changes mission state, logs reason, publishes updates |
| `publishMissionState()` | Publishes current state to `/wordle_bot/mission_state` |
| `publishPerceptionState()` | Publishes state to `/mission/state` (for Perception subsystem) |

#### Topic Subscriptions (Inputs)

| Topic | Type | Source | Purpose |
|-------|------|--------|---------|
| `/perception/human_detected` | `std_msgs/Bool` | Perception | Safety interrupt |
| `/perception/status` | `std_msgs/String` | Perception | Scan progress ("SCANNING", "IDLE") |
| `/perception/detections` | `std_msgs/String` | Perception | JSON list of detected letters + poses |
| `/wordle_bot/motion_cmd` | `std_msgs/String` | Operator/GUI | Commands: START, STOP, HOME |
| `/wordle_bot/motion_complete` | `std_msgs/Bool` | Motion Planning | Feedback when arm finishes move |
| `/gamification/mission_state` | `std_msgs/String` | Gamification | JSON: guess word + per-letter pick poses |

#### Topic Publications (Outputs)

| Topic | Type | Destination | Purpose |
|-------|------|-------------|---------|
| `/mission/state` | `std_msgs/String` | Perception, Gamification | "SCANNING" / "IDLE" mode switch |
| `/wordle_bot/mission_state` | `std_msgs/String` | GUI | Human-readable state (IDLE, SCANNING, MOVING, etc) |
| `/wordle_bot/mission_progress` | `std_msgs/String` | GUI | JSON step-by-step progress for visualization |
| `/wordle_bot/set_mission` | `geometry_msgs/PoseArray` | Motion Planning | Waypoints to visit in sequence |
| `/wordle_bot/start_mission` | `std_msgs/Bool` | Motion Planning | Trigger arm to execute queued waypoints |
| `/wordle_bot/stop_mission` | `std_msgs/Bool` | Motion Planning | Emergency halt (human detected) |
| `/wordle_bot/resume_mission` | `std_msgs/Bool` | Motion Planning | Resume after human leaves |
| `/wordle_bot/abort_mission` | `std_msgs/Bool` | Motion Planning | Abort and return to safe pose |

---

### 2. **MainWindow** (The Qt GUI)

**File:** [interaction_execution/src/main_window.cpp](interaction_execution/src/main_window.cpp)
**Header:** [interaction_execution/include/interaction_execution/main_window.hpp](interaction_execution/include/interaction_execution/main_window.hpp)

#### Purpose
Provides the operator interface with:
- Real-time camera feed from RealSense
- RViz simulation view
- MoveIt motion planning preview
- Wordle game display
- Mission diagnostics and event log
- Voice control integration
- Safety status banner

#### Key Components

**View System:**
- `CameraView`: Displays RealSense RGB/depth stream with CV overlays
- `RvizSimView`: RViz display for full system simulation
- `RvizMoveItView`: MoveIt motion planning interface
- `WordleView`: Current guess word, attempt count, candidate filters
- Can switch between views via drawer navigation

**Diagnostics System:**
```cpp
QPlainTextEdit * diagnostics_event_log_              // Event timeline
QPlainTextEdit * diagnostics_mission_json_view_      // Mission JSON details
QPlainTextEdit * diagnostics_game_json_view_         // Gamification state
QLabel * diagnostics_mission_value_label_            // State summary
QLabel * diagnostics_safety_value_label_             // Safety status
QLabel * diagnostics_perception_value_label_         // Perception status
QLabel * diagnostics_wordle_value_label_             // Game attempt info
```

**Safety Controls:**
- Large red "STOP" button for emergency halt
- Displays human detection status
- Banner color changes (red if unsafe, green if safe)

**Voice Integration:**
- Spawns `voice_helper_process_` (Python subprocess)
- Records operator feedback (Green/Yellow/Gray feedback for Wordle)
- Can manually override voice recognition

**Gamification Bridge:**
- Receives current guess from `/gamification/guess`
- Sends feedback (G/Y/B) to `/gamification/feedback`
- Shows word difficulty (candidate count)
- Can set secret word and game mode

#### Topic Subscriptions (Inputs)

| Topic | Type | Purpose |
|-------|------|---------|
| `/perception/human_detected` | `std_msgs/Bool` | Update safety banner |
| `/mission/state` | `std_msgs/String` | Display perception mode |
| `/perception/status` | `std_msgs/String` | Show scan progress |
| `/perception/detections` | `std_msgs/String` | Display detected letters |
| `/wordle_bot/mission_state` | `std_msgs/String` | Show current mission state |
| `/wordle_bot/mission_progress` | `std_msgs/String` | Render step-by-step progress |
| `/gamification/guess` | `std_msgs/String` | Show current guess word |
| `/diagnostics` | `std_msgs/String` | Game state (candidates, attempts) |

#### Topic Publications (Outputs)

| Topic | Type | Purpose |
|-------|------|---------|
| `/mission/state` | `std_msgs/String` | Send mission state (from voice or button) |
| `/wordle_bot/mission_cmd` | `std_msgs/String` | Operator commands: START, STOP, HOME |
| `/gamification/feedback` | `std_msgs/String` | G/Y/B feedback (manual or voice) |
| `/gamification/mode` | `std_msgs/String` | Set game mode (ModeA, ModeB) |
| `/gamification/secret_word` | `std_msgs/String` | Set the word to guess |
| `/gamification/player_guess` | `std_msgs/String` | Manual guess entry |

#### Layout Structure

```
┌─────────────────────────────────────────────────┐
│ TopBar: Safety Status Banner                    │
│ (Red if human detected, green if safe)          │
└─────────────────────────────────────────────────┘
┌──────────┬───────────────────────────────────────┐
│ Drawer   │ Content Stack (tabs switch views)     │
│ (Collapsed)                                     │
│ - Sim    │ ┌─────────────────────────────────┐  │
│ - MoveIt │ │  RViz Sim View (default)        │  │
│ - Camera │ │  (or Camera / MoveIt / Wordle)  │  │
│ - Help   │ └─────────────────────────────────┘  │
│ - Diag   │                                       │
└──────────┴───────────────────────────────────────┘
Diagnostics Window (spawned separately):
- Mission Progress (step visualization)
- Event Log (with timestamps)
- Mission JSON Viewer
- Game State Viewer
```

---

### 3. **Supporting View Classes**

#### CameraView ([interaction_execution/src/camera_view.cpp](interaction_execution/src/camera_view.cpp))
- Subscribes to RealSense image streams
- Renders RGB + depth with OpenCV overlay
- Shows detected letter bounding boxes
- Toggles between raw and CV-processed modes

#### RvizSimView ([interaction_execution/src/rviz_sim_view.cpp](interaction_execution/src/rviz_sim_view.cpp))
- Embeds RViz inside Qt
- Shows robot, board, and detected letter positions
- Reads TF tree and robot state

#### RvizMoveItView ([interaction_execution/src/rviz_moveit_view.cpp](interaction_execution/src/rviz_moveit_view.cpp))
- MoveIt motion planning visualization
- Shows collision objects and planned trajectories
- Allows interactive goal dragging (if MoveIt supports it)

#### WordleView ([interaction_execution/src/wordle_view.cpp](interaction_execution/src/wordle_view.cpp))
- Displays the current 5-letter guess
- Shows attempt count and remaining candidates
- Shows feedback from prior attempts (green=correct, yellow=wrong spot, gray=not in word)

---

## Data Flow: A Complete Mission Cycle

### Flow Diagram
```
1. Operator presses "START" button in GUI
   └─ MainWindow publishes /wordle_bot/mission_cmd = "START"

2. MissionCoordinator receives "START"
   └─ transitionTo(SCANNING)
   └─ Publishes /mission/state = "SCANNING"

3. Perception receives /mission/state = "SCANNING"
   └─ Starts CNN letter classification
   └─ Publishes /perception/detections (JSON with letters + 3D poses)

4. MissionCoordinator receives detections
   └─ Counts detected letters
   └─ If count >= minimum (default: 1), transitionTo(READY_TO_MOVE)

5. Gamification receives /perception/detections
   └─ Selects best next word guess
   └─ Publishes /gamification/mission_state (JSON: word + per-letter pick poses)

6. MissionCoordinator receives /gamification/mission_state
   └─ Parses JSON and builds PoseArray
   └─ Publishes /wordle_bot/set_mission (PoseArray with 5 pick poses)
   └─ Publishes /wordle_bot/start_mission = true
   └─ Transition to MOVING

7. Motion Planning receives set_mission + start_mission
   └─ Plans and executes pick-and-place for each letter
   └─ Publishes /wordle_bot/motion_complete after each letter
   └─ Publishes /wordle_bot/mission_complete when all letters placed

8. MissionCoordinator receives motion_complete (per letter)
   └─ Increments step counter
   └─ Publishes /wordle_bot/mission_progress (JSON with step details)
   └─ GUI updates progress visualization

9. MissionCoordinator receives mission_complete
   └─ transitionTo(IDLE)
   └─ Ready for operator feedback cycle

10. Operator provides G/Y/B feedback (voice or manual)
    └─ MainWindow publishes /gamification/feedback

11. Gamification filters candidate list
    └─ Publishes /diagnostics with updated state
    └─ Loop returns to step 1 (press START again)
```

### Safety Interrupt Flow (Parallel Every Frame)
```
Perception detects human
└─ Publishes /perception/human_detected = true

MissionCoordinator.tickSafetyGuard() runs
└─ Publishes /wordle_bot/stop_mission = true
└─ transitionTo(STOPPED)

GUI shows red safety banner "HUMAN DETECTED - MOTION HALTED"

When human leaves:
└─ Perception publishes /perception/human_detected = false

MissionCoordinator detects change
└─ Publishes /wordle_bot/resume_mission = true
└─ Transitions back to prior state (MOVING or SCANNING)
```

---

## ROS2 Launch

**File:** [interaction_execution/launch/](interaction_execution/launch/)

The interaction execution package launches two separate executables:

### 1. `mission_coordinator_node`
- **Executable:** `mission_coordinator_node` (from [mission_coordinator_main.cpp](interaction_execution/src/mission_coordinator_main.cpp))
- **Runs:** MissionCoordinator behavior tree + pub/sub logic
- **Parameters (from config files):**
  - `auto_dispatch_motion`: Whether to auto-trigger motion (default: false)
  - `minimum_detected_blocks`: Min letter count before scanning complete (default: 1)
  - `goal.x`, `goal.y`, `goal.z`: Task goal position
  - `home.x`, `home.y`, `home.z`: Home safe position
  - `goal_frame_id`, `home_frame_id`: Reference frames

### 2. `interaction_execution_node`
- **Executable:** `interaction_execution_node` (from [main.cpp](interaction_execution/src/main.cpp))
- **Runs:** Qt GUI + camera view + RViz integration
- **Dependencies:**
  - Qt5, RViz (visualizations)
  - OpenCV (camera stream processing)
  - ROS2 RCL (ROS client library)

---

## Build Configuration

**CMakeLists.txt:** [interaction_execution/CMakeLists.txt](interaction_execution/CMakeLists.txt)

```cmake
# Two executables compiled:
add_executable(interaction_execution_node ...)    # GUI
add_executable(mission_coordinator_node ...)       # Behavior tree

# Qt magic (auto MOC, UIC, RCC)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenCV REQUIRED)
```

---

## Interface Points with Other Subsystems

### **← Perception Subsystem**
- **Read:** `/perception/human_detected`, `/perception/status`, `/perception/detections`
- **Write:** `/mission/state` (SCANNING/IDLE signal to wake/sleep perception)
- **Purpose:** Get letter detections, monitor human safety

### **→ Gamification Subsystem**
- **Read:** `/gamification/mission_state` (JSON: guess word + pick poses)
- **Read:** `/diagnostics` (board state for GUI)
- **Write:** `/mission/state` (START/STOP for game lifecycle)
- **Purpose:** Orchestrate word selection, get per-letter pick locations

### **→ Motion Planning & Control**
- **Write:** `/wordle_bot/set_mission` (PoseArray waypoints)
- **Write:** `/wordle_bot/start_mission`, `/wordle_bot/stop_mission`, `/wordle_bot/resume_mission`, `/wordle_bot/abort_mission`
- **Read:** `/wordle_bot/motion_complete`, `/wordle_bot/mission_complete`
- **Purpose:** Send arm commands, receive motion feedback

### **← Voice Control**
- **Via:** Qt subprocess bridge (separate Python process)
- **Purpose:** Collect operator G/Y/B feedback, issue voice commands

---

## Key Design Patterns

### 1. **Behavior Tree (Manual Implementation)**
Rather than using a formal BT library, the coordinator implements tree logic manually:
- Each branch is a method: `tickSafetyGuard()`, `tickCommandBranch()`, etc.
- Each returns `NodeStatus`: FAILURE, SUCCESS, RUNNING
- Tree executes in sequence every 500ms via timer callback

### 2. **State Machine with Clear Transitions**
- Explicit state enum (IDLE, SCANNING, READY_TO_MOVE, MOVING, etc.)
- `transitionTo()` method logs reason and publishes updates
- Safety guards prevent invalid transitions

### 3. **Message-Based Coordination**
- No direct function calls between subsystems
- Everything via ROS2 topics/services
- Decouples timing and execution

### 4. **Heartbeat Timer for Polling**
- 500ms timer calls `tickTree()`
- Allows state changes to be detected within 500ms
- Prevents spinning CPU

### 5. **JSON for Structured Data**
- Complex payloads (detections, mission state) sent as JSON strings
- Humans can read logs easily
- Light-weight vs. custom message types

---

## Testing & Debugging

### Run Just the Mission Coordinator
```bash
ros2 run interaction_execution mission_coordinator_node
```

### Run Just the GUI
```bash
ros2 run interaction_execution interaction_execution_node
```

### Monitor Topics in Real-Time
```bash
ros2 topic echo /mission/state                    # Perception mode
ros2 topic echo /wordle_bot/mission_state         # Mission state
ros2 topic echo /perception/detections            # Letter detections
ros2 topic echo /wordle_bot/motion_complete       # Motion feedback
```

### Check Parameters
```bash
ros2 param list mission_coordinator               # List all params
ros2 param get mission_coordinator auto_dispatch_motion
```

---

## Summary

The **Interaction Execution** subsystem is the **brain and face** of the RS2 WordleBot:
- **Brain (MissionCoordinator):** Orchestrates mission lifecycle via behavior tree, coordinates all subsystems
- **Face (MainWindow + Views):** Provides operator interface with real-time feedback and diagnostics

It integrates tightly with:
- **Perception** for human safety and letter detection
- **Gamification** for word selection and game state
- **Motion Planning** for arm execution

All communication is ROS2 topic-based, enabling independent operation and testing of each subsystem.
