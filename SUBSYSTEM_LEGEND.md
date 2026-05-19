# RS2 Subsystem Legend

This document describes the four subsystems of the RS2 WordleBot project, their internal processes, and how they interact with each other via ROS2 topics and services.

---

## Table of Contents
1. [System Overview](#system-overview)
2. [Interaction & Execution](#1-interaction--execution)
3. [Gamification](#2-gamification)
4. [Perception](#3-perception)
5. [Motion Planning & Control](#4-motion-planning--control)
6. [Cross-Subsystem Data Flow](#cross-subsystem-data-flow)
7. [Topic Reference Table](#topic-reference-table)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                     INTERACTION & EXECUTION                         │
│             MissionCoordinator (Behavior Tree) + Qt GUI             │
│                                                                     │
│  Orchestrates the full mission lifecycle. Reads sensor and game     │
│  state, enforces safety, issues commands to all other subsystems.   │
└────┬──────────────────────┬──────────────────────┬──────────────────┘
     │ /mission/state        │ /wordle_bot/          │ reads
     │ (START/STOP/SCAN)     │  set_mission          │ /perception/*
     ▼                       ▼                       │ /gamification/*
┌──────────────┐   ┌─────────────────────┐           │
│ GAMIFICATION │   │ MOTION PLANNING &   │◄──────────┘
│              │   │ CONTROL             │
│ Word solver  │   │ MoveIt pick & place │
│ + Voice ctrl │   │ UR3e arm execution  │
└──────┬───────┘   └─────────────────────┘
       │ /perception/detections (letters + 3D poses)
       ▼
┌──────────────┐
│  PERCEPTION  │
│              │
│ RealSense    │
│ CNN + Depth  │
└──────────────┘
```

---

## 1. Interaction & Execution

**Packages:** `interaction_execution`
**Language:** C++ (nodes), Python (UI bridge)
**Key Files:** [mission_coordinator.cpp](interaction_execution/src/mission_coordinator.cpp), [main_window.cpp](interaction_execution/src/main_window.cpp)

### Key Processes

| Process | Description |
|---|---|
| `MissionCoordinator` | Behavior tree node that orchestrates the full game loop |
| `SafetyGuard` branch | Halts all motion if a human is detected in the workspace |
| `CommandBranch` | Processes manual operator commands (START / STOP / HOME) |
| `ScanBranch` | Triggers perception to scan the letter board |
| `MotionBranch` | Monitors motion completion and advances the mission state |
| Qt GUI | Displays camera feed, RViz sim, diagnostics, and operator controls |

### Mission States

`IDLE` → `SCANNING` → `READY_TO_MOVE` → `MOVING` → `IDLE` (loop)
Also: `STOPPED`, `HOMING`, `ERROR`

### Inputs (Subscribed Topics)

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/perception/human_detected` | `std_msgs/Bool` | Perception | Safety: halt if human present |
| `/perception/status` | `std_msgs/String` | Perception | Know when scanning is active |
| `/perception/detections` | `std_msgs/String` | Perception | Read detected letters for GUI |
| `/wordle_bot/motion_complete` | `std_msgs/Bool` | Motion Planning | Know when arm finishes a move |
| `/wordle_bot/mission_cmd` | `std_msgs/String` | Operator / GUI | Manual START / STOP / HOME |
| `/gamification/mission_state` | `std_msgs/String` | Gamification | JSON letter placements for mission |
| `/diagnostics` | `std_msgs/String` | Gamification | Board state for GUI display |

### Outputs (Published Topics)

| Topic | Type | Destination | Purpose |
|---|---|---|---|
| `/mission/state` | `std_msgs/String` | Perception, Gamification | Drive scan/play lifecycle |
| `/wordle_bot/set_mission` | `geometry_msgs/PoseArray` | Motion Planning | Send waypoint sequence |
| `/wordle_bot/start_mission` | `std_msgs/Bool` | Motion Planning | Trigger arm execution |
| `/wordle_bot/stop_mission` | `std_msgs/Bool` | Motion Planning | Halt arm |
| `/wordle_bot/resume_mission` | `std_msgs/Bool` | Motion Planning | Resume halted arm |
| `/wordle_bot/abort_mission` | `std_msgs/Bool` | Motion Planning | Abort and recover |
| `/wordle_bot/mission_progress` | `std_msgs/String` | GUI | Current step details (JSON) |
| `/wordle_bot/mission_state` | `std_msgs/String` | GUI | Human-readable mission state |

---

## 2. Gamification

**Packages:** `gamification`, `voice_control`
**Language:** Python
**Key Files:** [gamification_node.py](gamification/gamification_node.py), [wordle_logic.py](gamification/wordle_logic.py), [voice_node.py](voice_control/voice_node.py)

### Key Processes

| Process | Description |
|---|---|
| `GamificationNode` | ROS2 node that manages Wordle solving logic and game state |
| Opening word selection | Picks a weighted-random opener from a curated list on attempt 1 |
| Candidate filtering | Narrows the word list using G/B/I feedback after each guess |
| Letter frequency scoring | Ranks remaining candidates by aggregate letter frequency |
| Mission state publishing | Converts guess + available letter positions into a pick-and-place sequence |
| `voice_node.py` | Accepts spoken or typed G/B/I feedback; relays voice commands as `/mission/state` |
| Speaker verification | Optional speaker ID gate before accepting voice commands |

### Services Provided

| Service | Type | Purpose |
|---|---|---|
| `/gamification/reset` | `std_srvs/Trigger` | Reset board and candidate list |
| `/gamification/undo` | `std_srvs/Trigger` | Undo last attempt |

### Inputs (Subscribed Topics)

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/perception/detections` | `std_msgs/String` | Perception | Available letters + 3D positions |
| `/mission/state` | `std_msgs/String` | Interaction & Exec | START / STOP / RESET game |
| `/gamification/feedback` | `std_msgs/String` | Voice Control / GUI | G/B/I feedback per letter |

### Outputs (Published Topics)

| Topic | Type | Destination | Purpose |
|---|---|---|---|
| `/gamification/guess` | `std_msgs/String` | Voice Control, GUI | Current 5-letter guess |
| `/gamification/mission_state` | `std_msgs/String` | Interaction & Exec | JSON: word + per-letter pick poses |
| `/diagnostics` | `std_msgs/String` | Interaction & Exec (GUI) | Board state, candidates, attempt count |

### Voice Control Sub-Process

| Direction | Topic | Purpose |
|---|---|---|
| **In** | `/gamification/guess` | Read out the current guess |
| **In** | `/mission/state` | Know game phase |
| **Out** | `/gamification/feedback` | Publish recognized G/B/I tokens |
| **Out** | `/mission/state` | Issue voice-activated START / STOP |

---

## 3. Perception

**Package:** `perception`
**Language:** Python
**Key Files:** [realsense_camera_cnn.py](perception/realsense_camera_cnn.py), [train_letter_cnn.py](perception/train_letter_cnn.py)

### Key Processes

| Process | Description |
|---|---|
| `RealSenseCNNNode` | Main perception node; runs all CV and publishes results |
| MediaPipe pose detection | Per-frame human presence check for workspace safety |
| Block / card detection | Brightness or adaptive threshold + depth gate to isolate letter tiles |
| CNN letter classification | 3-layer CNN (32→64→128 filters), 64×64 grayscale input, 36-class output (A–Z, 0–9) |
| Temporal majority voting | Smooths CNN output over 15 frames to reduce flicker |
| 3D pose estimation | Uses RealSense intrinsics + aligned depth to compute x/y/z in metres and rotation angle |

### Inputs (Subscribed Topics)

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense driver | RGB video stream |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense driver | Aligned depth frames |
| `/mission/state` | `std_msgs/String` | Interaction & Exec | SCANNING / IDLE mode switch |

### Outputs (Published Topics)

| Topic | Type | Destination | Purpose |
|---|---|---|---|
| `/perception/human_detected` | `std_msgs/Bool` | Interaction & Exec | Safety signal, published every frame |
| `/perception/status` | `std_msgs/String` | Interaction & Exec | Current mode (SCANNING / IDLE) |
| `/perception/detections` | `std_msgs/String` | Gamification, Interaction & Exec | JSON list of detected letters with 3D poses |

### Detection JSON Format

```json
{
  "blocks": [
    {
      "letter": "A",
      "conf": 94.2,
      "x_m": 0.0412,
      "y_m": -0.0231,
      "z_m": 0.3820,
      "theta_deg": 12.5
    }
  ]
}
```

---

## 4. Motion Planning & Control

**Package:** `wordlebot_control`
**Language:** C++
**Key Files:** [wordle_bot_control_node.cpp](motion_planning_and_control/wordlebot_control/src/wordle_bot_control_node.cpp), [wordle_bot_controller.cpp](motion_planning_and_control/wordlebot_control/src/wordle_bot_controller.cpp)
**Dependencies:** MoveIt 2, MoveIt Task Constructor, TF2, UR3e driver

### Key Processes

| Process | Description |
|---|---|
| `WordleBotControlNode` | Top-level ROS2 node; manages mission queue and lifecycle |
| Mission queue | Buffers incoming waypoint goals and executes them in order |
| Waypoint mode | Moves the arm through a sequence of `PoseStamped` targets |
| Pick-and-place mode | Receives per-letter pick poses + slot index; executes full grasp cycle |
| Collision object injection | Adds 50 mm cube collision objects at pick locations for MoveIt planning |
| MoveIt Task Constructor | Generates and executes pick/place motion primitives |

### Custom Message

```
wordlebot_control/PickPlaceTask
  geometry_msgs/PoseStamped pick_pose
  int32 place_slot          # 1–5, maps to predefined board slot positions
```

### Inputs (Subscribed Topics)

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/wordle_bot/set_mission` | `geometry_msgs/PoseArray` | Interaction & Exec | Waypoint sequence for current word |
| `/wordle_bot/start_mission` | `std_msgs/Bool` | Interaction & Exec | Begin executing queued mission |
| `/wordle_bot/stop_mission` | `std_msgs/Bool` | Interaction & Exec | Halt arm immediately |
| `/wordle_bot/resume_mission` | `std_msgs/Bool` | Interaction & Exec | Resume halted mission |
| `/wordle_bot/abort_mission` | `std_msgs/Bool` | Interaction & Exec | Abort and recover to safe pose |
| `/wordle_bot/add_collision_object` | `moveit_msgs/CollisionObject` | Internal / external | Inject dynamic obstacle |
| `/wordle_bot/goal_pose` | `geometry_msgs/PoseStamped` | Legacy | Single-goal interface |
| `perception/letter_objects` | `wordlebot_control/PickPlaceTask` | Perception (direct) | Per-letter pick task |

### Outputs (Published Topics)

| Topic | Type | Destination | Purpose |
|---|---|---|---|
| `/wordle_bot/motion_complete` | `std_msgs/Bool` | Interaction & Exec | Single move finished |
| `/wordle_bot/goal_reached` | `std_msgs/Bool` | Interaction & Exec | Per-waypoint completion |
| `/wordle_bot/mission_complete` | `std_msgs/Bool` | Interaction & Exec | Full word placed |
| `/wordle_bot/robot_state` | `std_msgs/String` | GUI | "RUNNING" or "IDLE" |

---

## Cross-Subsystem Data Flow

Below is the end-to-end flow for one Wordle attempt:

```
Operator presses START
        │
        ▼
[Interaction & Execution]
  Publishes /mission/state = "SCANNING"
        │
        ▼
[Perception]
  Scans board, classifies letters via CNN
  Publishes /perception/detections (JSON letter + 3D poses)
        │
        ├──────────────────────────────────────────────┐
        ▼                                              ▼
[Gamification]                              [Interaction & Exec GUI]
  Receives available letters                  Displays detections
  Selects best guess word
  Publishes /gamification/mission_state
    (JSON: word + per-letter pick poses)
        │
        ▼
[Interaction & Execution]
  Converts mission_state → PoseArray
  Publishes /wordle_bot/set_mission
  Publishes /wordle_bot/start_mission = true
        │
        ▼
[Motion Planning & Control]
  Executes pick-and-place for each letter
  Publishes /wordle_bot/motion_complete per move
  Publishes /wordle_bot/mission_complete when word is placed
        │
        ▼
[Interaction & Execution]
  Mission complete → triggers feedback collection
        │
        ▼
[Voice Control]
  Operator speaks / types G/B/I feedback
  Publishes /gamification/feedback
        │
        ▼
[Gamification]
  Filters candidate list
  Ready for next attempt → loop back to SCANNING
```

**Safety interrupt** (runs in parallel every frame):

```
[Perception] → /perception/human_detected = true
      │
      ▼
[Interaction & Execution] SafetyGuard
  Publishes /wordle_bot/stop_mission
  Halts until human_detected = false
```

---

## Topic Reference Table

| Topic | Publisher | Subscriber(s) | Type |
|---|---|---|---|
| `/mission/state` | Interaction & Exec, Voice Control | Perception, Gamification | `std_msgs/String` |
| `/perception/detections` | Perception | Gamification, Interaction & Exec | `std_msgs/String` |
| `/perception/human_detected` | Perception | Interaction & Exec | `std_msgs/Bool` |
| `/perception/status` | Perception | Interaction & Exec | `std_msgs/String` |
| `/gamification/guess` | Gamification | Voice Control, GUI | `std_msgs/String` |
| `/gamification/mission_state` | Gamification | Interaction & Exec | `std_msgs/String` |
| `/gamification/feedback` | Voice Control / GUI | Gamification | `std_msgs/String` |
| `/diagnostics` | Gamification | Interaction & Exec (GUI) | `std_msgs/String` |
| `/wordle_bot/set_mission` | Interaction & Exec | Motion Planning | `geometry_msgs/PoseArray` |
| `/wordle_bot/start_mission` | Interaction & Exec | Motion Planning | `std_msgs/Bool` |
| `/wordle_bot/stop_mission` | Interaction & Exec | Motion Planning | `std_msgs/Bool` |
| `/wordle_bot/resume_mission` | Interaction & Exec | Motion Planning | `std_msgs/Bool` |
| `/wordle_bot/abort_mission` | Interaction & Exec | Motion Planning | `std_msgs/Bool` |
| `/wordle_bot/motion_complete` | Motion Planning | Interaction & Exec | `std_msgs/Bool` |
| `/wordle_bot/goal_reached` | Motion Planning | Interaction & Exec | `std_msgs/Bool` |
| `/wordle_bot/mission_complete` | Motion Planning | Interaction & Exec | `std_msgs/Bool` |
| `/wordle_bot/robot_state` | Motion Planning | GUI | `std_msgs/String` |
| `/wordle_bot/mission_progress` | Interaction & Exec | GUI | `std_msgs/String` |
| `/wordle_bot/mission_state` | Interaction & Exec | GUI | `std_msgs/String` |
| `/wordle_bot/mission_cmd` | Operator / GUI | Interaction & Exec | `std_msgs/String` |
| `/wordle_bot/add_collision_object` | External | Motion Planning | `moveit_msgs/CollisionObject` |
| `perception/letter_objects` | Perception (direct) | Motion Planning | `wordlebot_control/PickPlaceTask` |
| `/camera/camera/color/image_raw` | RealSense driver | Perception | `sensor_msgs/Image` |
| `/camera/camera/aligned_depth_to_color/image_raw` | RealSense driver | Perception | `sensor_msgs/Image` |
