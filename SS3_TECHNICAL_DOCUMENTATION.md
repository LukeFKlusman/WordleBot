# SS3 — Interaction & Execution

## 7.1 Purpose

The Interaction Execution subsystem serves as the **mission control layer and human interface** for the RS2 WordleBot system. It orchestrates the entire game lifecycle through a custom behavior tree (implemented as a state machine with parallel monitoring branches), manages human-robot safety, and provides operators with real-time diagnostics via a Qt-based GUI. The behavior tree runs continuously at 500ms intervals, constantly monitoring safety conditions while executing task sequences, enabling immediate interruption when humans enter the workspace or unexpected scene changes occur.

**Key Responsibilities:**
- **Mission Orchestration:** Coordinates perception, gamification, and motion subsystems through a behavior tree that transitions between IDLE → SCANNING → READY_TO_MOVE → MOVING states
- **Safety Monitoring:** Parallel branch continuously monitors human detection, scene changes, and unknown objects; interrupts any active task immediately if unsafe conditions detected
- **Operator Interface:** Qt GUI with real-time camera feed, robot simulation, diagnostics, and voice control integration
- **Failure Recovery:** Detects perception and motion timeouts, retries automatically up to configured limits, escalates to operator intervention only when recovery exhausted

## 7.2 Key Files

| **File** | **Purpose** |
| --- | --- |
| [interaction_execution/src/mission_coordinator.cpp](../interaction_execution/src/mission_coordinator.cpp) | Behavior tree executor (heartbeat-driven state machine), mission state transitions, command dispatch, failure detection |
| [interaction_execution/include/interaction_execution/mission_coordinator.hpp](../interaction_execution/include/interaction_execution/mission_coordinator.hpp) | Mission coordinator interface and state definitions |
| [interaction_execution/src/main_window.cpp](../interaction_execution/src/main_window.cpp) | Qt GUI main window, multi-view layout (Sim/Camera/MoveIt/Wordle/Diagnostics), mission status display |
| [interaction_execution/include/interaction_execution/main_window.hpp](../interaction_execution/include/interaction_execution/main_window.hpp) | MainWindow class interface |
| [interaction_execution/src/camera_view.cpp](../interaction_execution/src/camera_view.cpp) | RealSense RGB/depth stream rendering, detected letter overlay visualization |
| [interaction_execution/src/rviz_sim_view.cpp](../interaction_execution/src/rviz_sim_view.cpp) | Embedded RViz display for robot/board/letter positions |
| [interaction_execution/src/rviz_moveit_view.cpp](../interaction_execution/src/rviz_moveit_view.cpp) | MoveIt motion planning visualization |
| [interaction_execution/src/wordle_view.cpp](../interaction_execution/src/wordle_view.cpp) | Current guess, attempt counter, candidate feedback display |
| [interaction_execution/launch/gui.launch.py](../interaction_execution/launch/gui.launch.py) | Launch script for coordinator, GUI, and conditional subsystem startup |

## 7.3 ROS 2 Interface

### Publishers

| **Topic** | **Type** | **Description** |
| --- | --- | --- |
| `/mission/state` | std_msgs/String | State for perception subsystem: "SCANNING" or "IDLE" |
| `/wordle_bot/mission_state` | std_msgs/String | Current mission state: IDLE, SCANNING, READY_TO_MOVE, MOVING, STOPPED, etc. |
| `/wordle_bot/mission_progress` | std_msgs/String | JSON step-by-step progress: scan status, motion steps, safety flags |
| `/wordle_bot/set_mission` | geometry_msgs/PoseArray | Waypoints (per-letter pick poses) for motion subsystem |
| `/wordle_bot/start_mission` | std_msgs/Bool | Trigger signal to begin queued motion |
| `/wordle_bot/stop_mission` | std_msgs/Bool | Emergency stop signal (human intrusion, timeout) |
| `/wordle_bot/resume_mission` | std_msgs/Bool | Resume after safety pause (human left, safety cleared) |
| `/wordle_bot/abort_mission` | std_msgs/Bool | Abort current mission and return to safe pose |

### Subscribers

| **Topic** | **Type** | **Description** |
| --- | --- | --- |
| `/perception/human_detected` | std_msgs/Bool | Human intrusion flag from SS1 — triggers safety stop |
| `/perception/status` | std_msgs/String | Perception scan progress: "SCANNING", "IDLE", "ERROR" |
| `/perception/detections` | std_msgs/String | JSON: detected letter blocks with 3D poses and classifications |
| `/wordle_bot/mission_cmd` | std_msgs/String | Operator commands: START, STOP, HOME, RESUME, ABORT (from GUI or voice) |
| `/wordle_bot/motion_complete` | std_msgs/Bool | Motion subsystem signals completion of current goal |
| `/gamification/mission_state` | std_msgs/String | JSON: next guess word and per-letter pick poses from SS4 |
| `/diagnostics` | std_msgs/String | Game state: attempt count, candidates remaining, top guesses (from SS4) |

### Services

Not currently used; all coordination via latching/transient_local topics.

## 7.4 Behavior Tree Structure

### Tree Topology

The mission coordinator implements a **heartbeat-driven behavior tree** (500ms tick rate) with the following structure:

```
Root (Selector + Parallel Safety)
├── Safety Monitor Branch (Parallel, always running)
│   ├── Human Detection Check → STOPPED if true
│   ├── Scene Change Detection (optional) → Re-scan if detected
│   └── Unknown Object Flag (optional) → Warning or halt
│
├── Failure Detection Branch (Sequence)
│   ├── Perception Timeout Check (SCANNING state only)
│   │   └── Publish `/wordle_bot/stop_mission` if timeout
│   └── Motion Timeout Check (MOVING/HOMING state only)
│       └── Transition to MOTION_FAILED if timeout
│
├── Recovery Branch (Sequence)
│   ├── Safety Unlock (SAFETY_STOPPED → STOPPED when human cleared)
│   └── Scan Retry (RECOVERING → SCANNING if retries remain)
│
├── Command Branch (Selector)
│   ├── START: Transition to SCANNING (if safe)
│   ├── STOP: Transition to STOPPED
│   ├── HOME: Dispatch home goal and transition to HOMING
│   ├── RESUME: Transition from STOPPED back to prior motion-ready state
│   └── ABORT: Clear all pending goals, return to IDLE
│
├── Scan Branch (Sequence)
│   ├── SCANNING state handler
│   ├── Publish `/mission/state` = "SCANNING" to wake perception
│   └── Transition to READY_TO_MOVE when `hasEnoughDetections()` returns true
│
└── Motion Branch (Sequence)
    ├── READY_TO_MOVE → build PoseArray from gamification JSON
    ├── Publish `/wordle_bot/set_mission` with waypoints
    ├── Publish `/wordle_bot/start_mission` = true
    ├── Transition to MOVING
    └── Wait for `/wordle_bot/motion_complete` before returning to IDLE
```

### Tick Rate and Execution Model

- **Tick Frequency:** 500ms (hardcoded in mission_coordinator.cpp)
- **Execution Model:** Synchronous polling; each tick executes all branch methods in sequence
- **Blackboard:** State stored in member variables (mission state, pending commands, detection counts, timestamps)
- **Safety Priority:** `tickSafetyGuard()` runs first every tick; can interrupt any other branch immediately

### Fallback and Recovery Logic

| **Failure Scenario** | **Detection Method** | **Recovery Path** |
| --- | --- | --- |
| **Perception Timeout** | State elapsed > `perception_timeout_s` (default 10s) while SCANNING | Retry scanning up to `max_scan_retries` (default 1); then transition to PERCEPTION_FAILED |
| **Motion Timeout** | Motion elapsed > `motion_timeout_s` (default 20s) while MOVING/HOMING | Publish `/wordle_bot/stop_mission`, transition to MOTION_FAILED |
| **Human Intrusion** | `/perception/human_detected` = true | Publish `/wordle_bot/stop_mission`, transition to SAFETY_STOPPED; auto-clear to STOPPED after human leaves (if `AUTO_RESUME_AFTER_SAFETY` = true) |
| **Insufficient Detections** | Detection count < `minimum_detected_blocks` (default 1) | Remain in SCANNING until timeout or sufficient detections |

## 7.5 GUI

### Layout and Views

The Qt GUI features a **drawer + content stack + side panel** layout with 5 switchable views:

```
┌─────────────────────────────────────────────────────────────────┐
│ Safety Status Banner (Red if unsafe, green if safe)             │
├─────────────────────────────────────────────────────────────────┤
│ ┌────┬────────────────────────────┬──────────────────────────┐  │
│ │ ☰  │ Content Stack              │ Wordle Display           │  │
│ │    │ (RViz / Camera / MoveIt)   │ ┌────────────────────┐   │  │
│ │    │                            │ │ Guess: _____ (A4)  │   │  │
│ │Sim │                            │ │ Attempt: 2/6       │   │  │
│ │Cam │ [RViz Sim View (default)]  │ │ Candidates: 143    │   │  │
│ │Mov │                            │ │                    │   │  │
│ │Wrd │                            │ │ [G][B][I][B][G]    │   │  │
│ │Dgn │                            │ │ (Feedback legend)  │   │  │
│ │    │                            │ └────────────────────┘   │  │
│ │    │                            │ ┌────────────────────┐   │  │
│ │    │                            │ │ VOICE CONTROL      │   │  │
│ │    │                            │ │ [Record] [Confirm] │   │  │
│ │    │                            │ │ [Stop]   [Retry]   │   │  │
│ │    │                            │ └────────────────────┘   │  │
│ │    │                            │ ┌────────────────────┐   │  │
│ │    │                            │ │ SAFETY CONTROLS    │   │  │
│ │    │                            │ │ [START]   [STOP]   │   │  │
│ │    │                            │ │ [SCAN]    [HOME]   │   │  │
│ │    │                            │ └────────────────────┘   │  │
│ └────┴────────────────────────────┴──────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### View Descriptions

| **View** | **Content** |
| --- | --- |
| **Sim** (RViz) | Robot, gameboard, and detected letter 3D positions; TF tree visualization |
| **Camera** | RealSense RGB/depth stream with bounding boxes around detected letters |
| **MoveIt** | Motion planning trajectories, collision objects, joint states |
| **Wordle** | Current guess (linked to `/gamification/guess`), attempt counter, candidate feedback |
| **Diagnostics** | Mission state JSON, perception JSON, event log with timestamps |

### Controls and Publications

| **Control** | **Element** | **Action** | **Topic Published** | **Payload** |
| --- | --- | --- | --- | --- |
| Start | Button (Safety panel) | Begin scan | `/wordle_bot/mission_cmd` | `"START"` |
| Stop | Button (Safety panel) | Halt mission | `/wordle_bot/mission_cmd` | `"STOP"` |
| Scan Game Board | Button (Safety panel) | Rescan detections | `/wordle_bot/mission_cmd` | `"START"` |
| Home | Button (Safety panel) | Return arm to home | `/wordle_bot/mission_cmd` | `"HOME"` |
| Record | Voice button | Start recording | Voice subprocess stdin | Record signal |
| Confirm | Voice button | Confirm feedback entry | `/gamification/feedback` | Wordle feedback (e.g., "GBIBI") |
| Feedback Row | Per-letter toggles | Set G/B/I per position (manual) | `/gamification/feedback` | Feedback string |
| Mode Selector | Dropdown | Set game mode | `/gamification/mode` | "ModeA" or "ModeB" |
| Secret Word | Text field | Set the word to guess | `/gamification/secret_word` | Word string (5 letters, uppercase) |

## 7.6 Safety Monitor

The safety monitor is a **parallel branch of the behavior tree** that executes every 500ms tick, independent of task state.

### Safety Branch Logic

```cpp
NodeStatus MissionCoordinator::tickSafetyGuard() {
  // Always check human detection
  if (human_detected_) {
    if (state_ != MissionState::SAFETY_STOPPED && 
        state_ != MissionState::STOPPED &&
        state_ != MissionState::IDLE) {
      publishMissionSignal(stop_mission_pub_, "/wordle_bot/stop_mission");
      transitionTo(MissionState::SAFETY_STOPPED, "Human intrusion detected");
    }
    safety_stop_published_ = true;
  } else {
    // Human has left; unlock safety stop if conditions met
    if (state_ == MissionState::SAFETY_STOPPED && !human_detected_) {
      transitionTo(MissionState::STOPPED, "Safety cleared; awaiting operator RESUME or HOME");
    }
  }
  return human_detected_ ? NodeStatus::RUNNING : NodeStatus::SUCCESS;
}
```

### Trigger Conditions

| **Trigger** | **Source Topic** | **Condition** | **Action** |
| --- | --- | --- | --- |
| **Human Intrusion** | `/perception/human_detected` | Message.data == true | Publish `/wordle_bot/stop_mission`, transition to SAFETY_STOPPED |
| **Safety Clear** | `/perception/human_detected` | Message.data == false (sustained) | Transition to STOPPED, optionally auto-resume if `AUTO_RESUME_AFTER_SAFETY` = true |
| **Scene Change** | `/perception/scene_changed` | Block count delta > `SCENE_CHANGE_GRACE` | Abort in-progress motion, trigger re-scan (optional feature, not yet implemented) |
| **Unknown Object** | `/perception/out_of_category` | Confidence < threshold | Flag warning in GUI; optionally halt if `STRICT_UNKNOWN_HALT` = true (not yet implemented) |

### Recovery Flow

1. **Human Intrusion Detected:**
   - Current state (e.g., MOVING) → SAFETY_STOPPED
   - `/wordle_bot/stop_mission` published immediately
   - GUI displays red safety banner

2. **Human Leaves Workspace:**
   - `/perception/human_detected` = false received
   - SAFETY_STOPPED → STOPPED (no automatic resume by default)
   - Operator must press RESUME or HOME to continue

3. **Auto-Resume (if enabled):**
   - Config `AUTO_RESUME_AFTER_SAFETY` = true
   - Automatically transition from STOPPED back to prior motion-ready state
   - Useful for high-frequency transient intrusions

## 7.7 Mission Control & State Machine

### State Diagram

```
            [IDLE]
              ▲
              │ (START command)
              ▼
         [SCANNING] ◄──────────────────────┐
              │                            │
              │ (hasEnoughDetections)     │ (Retries remain)
              ▼                            │
       [READY_TO_MOVE]                    │
              │                            │
              │ (auto_dispatch + goal)    │
              ▼                            │
          [MOVING]                     [RECOVERING]
              │                    ▲       │
              │ (motion_complete)  │       │
              └────────────────────┼───────┤
                                   │       │
            [HOMING] ────────────┐ │       │
              │                  │ │       │
              │ (motion_complete)│ │       │
              └─────────────────►└─┴───────┘
                                   
    [SAFETY_STOPPED] ◄─── (human detected) ───┐
         │                                     │
         │ (human leaves)                      │
         ▼                                     │
      [STOPPED] ◄─── (STOP cmd) ──────────────┘
         │
         │ (RESUME or HOME cmd)
         └─────────────────────────────► [SCANNING] or [HOMING]

    [PERCEPTION_FAILED] ◄─── (timeout + retries exhausted)
    [MOTION_FAILED] ◄─── (motion timeout)
    [ERROR] ◄─── (unhandled exception)
```

### States and Transitions

| **State** | **Entry Condition** | **Exit Condition** | **Publications** |
| --- | --- | --- | --- |
| **IDLE** | Initial; after ABORT; mission_complete | START command | `/mission/state` = "IDLE" |
| **SCANNING** | START command (if safe) | `hasEnoughDetections()` true | `/mission/state` = "SCANNING" |
| **READY_TO_MOVE** | Scan completes with sufficient detections | `shouldAutoDispatch()` true OR manual task ready | `/wordle_bot/set_mission` (PoseArray) |
| **MOVING** | `/wordle_bot/start_mission` published | `/wordle_bot/motion_complete` received OR timeout | `/wordle_bot/start_mission` = true |
| **STOPPED** | STOP command OR human_detected cleared | RESUME/HOME command | (no publications) |
| **SAFETY_STOPPED** | human_detected = true while active | human_detected = false | `/wordle_bot/stop_mission` = true |
| **RECOVERING** | Perception timeout with retries remaining | Timeout cleared OR max retries exceeded | (retry detection) |
| **HOMING** | HOME command | motion_complete OR timeout | `/wordle_bot/set_mission` (home pose) |
| **PERCEPTION_FAILED** | Timeout after max retries | Manual RESUME/HOME command | (halted) |
| **MOTION_FAILED** | Motion timeout while MOVING/HOMING | Manual RESUME/HOME command | `/wordle_bot/stop_mission` = true |
| **ERROR** | Unhandled exception | Manual RESET/ABORT | (halted) |

## 7.8 Configurable Parameters

| **Parameter** | **Default** | **Scope** | **Description** |
| --- | --- | --- | --- |
| `auto_dispatch_motion` | false | mission_coordinator | If true, automatically transition READY_TO_MOVE → MOVING when motion goal is queued |
| `minimum_detected_blocks` | 1 | mission_coordinator | Min letter count to declare scan complete |
| `perception_timeout_s` | 10.0 | mission_coordinator | Seconds before scan is considered timed out |
| `max_scan_retries` | 1 | mission_coordinator | Number of retries before PERCEPTION_FAILED |
| `motion_timeout_s` | 20.0 | mission_coordinator | Seconds before motion is considered timed out |
| `goal.x`, `goal.y`, `goal.z` | (TBD) | mission_coordinator | Task goal position (parameterized via launch) |
| `home.x`, `home.y`, `home.z` | (TBD) | mission_coordinator | Home safe position |
| `goal_frame_id`, `home_frame_id` | "base_link" | mission_coordinator | Reference frames for goal poses |
| `AUTO_RESUME_AFTER_SAFETY` | true | hardcoded | Auto-transition SAFETY_STOPPED → STOPPED when human leaves |
| `SCENE_CHANGE_GRACE` | 1 | hardcoded | Block count delta tolerated before scene-change halt |
| `STRICT_UNKNOWN_HALT` | false | hardcoded | If true, unknown object halts mission; if false, warning only |

## 7.9 SS3 Limitations

- **Single-operator GUI:** No multi-user session support or role-based permissions
- **No formal behavior tree library:** Custom implementation limits reusability; state machine is imperative rather than declarative
- **Fixed tick rate:** 500ms heartbeat is hardcoded; not adaptive to event frequency
- **No blackboard persistence:** Behavior tree state is lost on node restart; requires manual operator RESET
- **Scene change detection not implemented:** Optional feature mentioned in design but not coded
- **Unknown object detection not implemented:** Optional feature mentioned in design but not coded
- **No watchdog on SS2 motion:** Relies on motion subsystem to publish timeout signals
- **Voice and GUI publish to same topics:** No arbitration for concurrent input from both sources (last one wins)
- **No mission state persistence:** Game progress not saved across restarts
- **ROS2 domain ID and RMW dependent:** Cross-machine communication requires matching domain and middleware config

## 7.10 Showcase Test Guide — Contract Criteria Walkthrough

This section provides step-by-step demonstrations for evaluation day, mapped to the grading criteria (P / C / D / HD). Each test can run with just the mission coordinator and GUI (no full system required for basic tests).

### Prerequisites

**Terminal 1 — Perception and Safety:**
```bash
cd ~/ros2_ws && source install/setup.bash
python3 ~/ros2_ws/src/RS2/perception/src/realsense_camera_cnn.py
```

**Terminal 2 — Mission Coordinator + GUI:**
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false
```

**Terminal 3 — Monitor Topics (for verification):**
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
```

### **Criterion P — Basic GUI Functionality**

**Demonstrate:** GUI launches and responds to operator commands.

| **Step** | **Action** |
| --- | --- |
| 1 | Launch GUI (Terminal 2 above) |
| 2 | Observe GUI displays safety panel with red "STOP" button and green "START" button |
| 3 | Press "START" button |
| 4 | Observe `/wordle_bot/mission_state` transitions to `SCANNING` |
| 5 | Press "STOP" button |
| 6 | Observe `/wordle_bot/mission_state` transitions to `STOPPED` |

**Expected Result:**
- GUI renders without crashes
- Safety controls panel is visible and responsive
- `/wordle_bot/mission_state` topics reflect button presses
- No unhandled exceptions in terminal output

**PASS:** The GUI provides basic mission control via START/STOP buttons, and the coordinator publishes state changes in response.

---

### **Criterion C — GUI Control with No Unintended Behaviors**

**Demonstrate:** State machine prevents invalid transitions and maintains consistent state.

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| 1 | Publish: `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"` | Simulator registers human_detected = false |
| 2 | Press "START" in GUI | `/wordle_bot/mission_state` = `SCANNING` |
| 3 | Publish: `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"` | Human detected while SCANNING |
| 4 | Check mission state immediately | `/wordle_bot/mission_state` = `SAFETY_STOPPED` (not `SCANNING`) |
| 5 | Publish: `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"` | Human leaves |
| 6 | Check mission state | `/wordle_bot/mission_state` = `STOPPED` (not auto-resume to SCANNING) |

**Why This Demonstrates C-Level:**
- No race conditions: safety interrupt takes precedence over scan state
- No jump to invalid states: SAFETY_STOPPED blocks intermediate transitions
- Safety condition must explicitly clear before recovery possible

**PASS:** The coordinator enforces valid state transitions; safety conditions prevent unsafe states from being entered.

---

### **Criterion D — Behavior Tree with Condition-Based Transitions**

#### **D1 — Perception Condition Triggers State Change**

| **Step** | **Action** | **Expected** |
| --- | --- | --- |
| 1 | Publish: `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"` | Safe to start |
| 2 | Press "START" | Mission state = `SCANNING` |
| 3 | Publish detections: `ros2 topic pub --once /perception/detections std_msgs/msg/String "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"` | Detections received |
| 4 | Check mission state | `/wordle_bot/mission_state` = `READY_TO_MOVE` (condition evaluated: 1 detection ≥ 1 minimum) |

**PASS:** The coordinator does not hard-code a state jump; it evaluates `hasEnoughDetections()` condition and only transitions when true.

#### **D2 — Failure Detection and Recovery (Perception Timeout)**

| **Step** | **Action** | **Expected** |
| --- | --- | --- |
| 1 | Launch with: `perception_timeout_s:=5.0 max_scan_retries:=1` | Timeout tuned for demo |
| 2 | Press "START" | Mission state = `SCANNING` |
| 3 | Do NOT publish `/perception/detections` | Simulate perception failure |
| 4 | Wait 6 seconds | Timeout threshold exceeded |
| 5 | Check mission state and logs | Should show RECOVERING → SCANNING (retry), then PERCEPTION_FAILED after max retries |

**Why This Demonstrates D:**
- Automatic failure detection (timeout-based)
- Condition-based recovery (retry counter checked)
- Escalation to manual intervention only when retries exhausted

**PASS:** The coordinator detects timeout, retries automatically, and escalates cleanly.

#### **D3 — Parallel Safety Monitoring (Interruption During Scan)**

| **Step** | **Action** | **Expected** |
| --- | --- | --- |
| 1 | Press "START", ensure state = `SCANNING` | Scan in progress |
| 2 | While SCANNING, publish: `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"` | Human enters workspace |
| 3 | Check mission state immediately (within 1 tick, ~500ms) | State = `SAFETY_STOPPED` (not stuck in SCANNING) |
| 4 | Monitor `/wordle_bot/stop_mission` in separate terminal | Should see `Bool(data=true)` published |

**Why This Demonstrates D:**
- Safety guard branch ticks in parallel, independent of scan state
- Interruption is not polled; it is detected every 500ms heartbeat
- Safety command propagates immediately to motion subsystem

**PASS:** Safety conditions interrupt any active branch immediately; safety branch has highest priority.

---

### **Criterion HD — Dynamic Context Switching and Sophisticated Recovery**

#### **HD1 — Continuous Monitoring While Motion Branch Active**

| **Step** | **Action** | **Expected** |
| --- | --- | --- |
| 1 | Reach MOVING state (requires motion subsystem + auto dispatch) | Motion in progress |
| 2 | While MOVING, publish: `ros2 topic pub --once /perception/detections std_msgs/msg/String "{data: '{\"blocks\":[{\"letter\":\"B\",\"x\":0.31,\"y\":0.11,\"z\":0.02}]}'}"` | Updated detections mid-motion |
| 3 | Monitor `/wordle_bot/mission_progress` | JSON should include updated detection info |
| 4 | Publish: `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"` | Safety interrupt during motion |
| 5 | Check mission state and logs | Should immediately transition to SAFETY_STOPPED; motion stops |

**Why This Demonstrates HD:**
- Multiple branches tick in parallel: motion and safety both active simultaneously
- Safety branch is not blocked by motion branch activity
- Perception updates are logged even while motion active
- Safety can interrupt at any time, context-agnostically

#### **HD2 — Priority-Based Task Selector (Safety > Motion > Scan)**

| **Step** | **Action** | **Expected** |
| --- | --- | --- |
| 1 | State = READY_TO_MOVE, motion goal queued | Ready to dispatch |
| 2 | Before motion dispatch completes, publish: `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"` | Safety condition while ready |
| 3 | Check behavior: does mission proceed to MOVING? | NO — safety priority prevents motion dispatch |
| 4 | Check `/wordle_bot/start_mission` topic | Should NOT be published; motion blocked by safety condition |

**Why This Demonstrates HD:**
- Three branch priorities encoded in tick order: SafetyGuard > FailureDetection > Recovery > Command > Scan > Motion
- Safety evaluated first; later branches skip if safety condition active
- Task priority is context-driven (which state we're in determines which branch is active)

#### **HD3 — Sophisticated Recovery Without Operator Intervention**

| **Step** | **Action** | **Expected** |
| --- | --- | --- |
| 1 | Trigger perception timeout (see D2): SCANNING → RECOVERING → SCANNING (retry) | Retry in progress |
| 2 | During retry, publish corrected detections | Detections received on second attempt |
| 3 | Check mission state | Should transition SCANNING → READY_TO_MOVE automatically (no operator input) |
| 4 | Verify `/wordle_bot/mission_progress` shows retry count | Progress JSON includes "retry_count": 1 |

**SEMI-PASS or PASS:** Recovery loop operates autonomously without operator intervention until max retries exhausted. (Note: If max retries <= 0 or timeout is too short, may not be testable in demo; document limitation.)

---

### Quick Diagnostics Table

| **Symptom** | **Likely Cause** | **Fix** |
| --- | --- | --- |
| GUI does not start | Qt library missing or display not set | Check `QT_QPA_PLATFORM=xcb` is set; ensure X11 forwarding if remote |
| Mission state stays IDLE after START | Perception not publishing human_detected = false | Manually publish `/perception/human_detected false` first |
| SCANNING state does not transition to READY_TO_MOVE | No detections published OR detection count < minimum | Publish test detections with valid JSON; check `minimum_detected_blocks` param |
| Motion branch never executes | `auto_dispatch_motion` = false | Either launch with `auto_dispatch_motion:=true` or manually trigger motion subsystem |
| Safety interrupt does not work | Perception subprocess crashed | Restart perception script in Terminal 1; verify human_detected topic is publishing |
| GUI shows stale mission state | Coordinator crashed or heartbeat paused | Check Terminal 2 for exceptions; restart mission_coordinator_node |

