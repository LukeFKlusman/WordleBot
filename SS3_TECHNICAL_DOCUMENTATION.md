# SS3 - Interaction & Execution

## 7.1 Purpose

The Interaction Execution subsystem is the **mission control and operator interface layer** for the RS2 WordleBot system. It contains:

- A `mission_coordinator_node` that runs a heartbeat-driven, behavior-tree-style state machine every 500 ms.
- A Qt GUI (`interaction_execution_node`) that provides operator controls, RViz/camera/task views, Wordle controls, voice helper integration, and diagnostics.

The current implementation coordinates perception state, human-detection safety stops, scan readiness, configured robot goal dispatch, and motion completion monitoring. It does not currently parse gamification mission JSON into per-letter pick-and-place paths; the coordinator dispatches configured task/home poses from ROS parameters.

**Key Responsibilities:**
- **Mission orchestration:** Tracks coordinator states such as IDLE, SCANNING, READY_TO_MOVE, MOVING, STOPPED, SAFETY_STOPPED, RECOVERING, HOMING, PERCEPTION_FAILED, and MOTION_FAILED.
- **Safety monitoring:** Checks `/perception/human_detected` at the top of every coordinator tick and publishes `/wordle_bot/stop_mission` when a human is detected.
- **Operator interface:** Provides a Qt GUI with RViz simulation, task visualization, camera view, diagnostics, Wordle controls, safety controls, and voice helper controls.
- **Failure recovery:** Supports perception timeout retry when `perception_timeout_s > 0.0`, plus motion timeout handling while waiting for `/wordle_bot/mission_complete`.

## 7.2 Key Files

| **File** | **Purpose** |
| --- | --- |
| [interaction_execution/src/mission_coordinator.cpp](interaction_execution/src/mission_coordinator.cpp) | Heartbeat-driven coordinator, state transitions, command handling, safety stop handling, timeout handling, configured goal dispatch |
| [interaction_execution/include/interaction_execution/mission_coordinator.hpp](interaction_execution/include/interaction_execution/mission_coordinator.hpp) | Coordinator interface, state enum, subscriptions, publishers, and internal state |
| [interaction_execution/src/main_window.cpp](interaction_execution/src/main_window.cpp) | Qt GUI main window, drawer navigation, safety controls, Wordle/gamification bridge, diagnostics, voice helper integration |
| [interaction_execution/include/interaction_execution/main_window.hpp](interaction_execution/include/interaction_execution/main_window.hpp) | `MainWindow` interface and GUI state |
| [interaction_execution/src/camera_view.cpp](interaction_execution/src/camera_view.cpp) | Camera stream rendering and mode switching |
| [interaction_execution/src/rviz_sim_view.cpp](interaction_execution/src/rviz_sim_view.cpp) | Embedded RViz display |
| [interaction_execution/src/hl_digital_twin_view.cpp](interaction_execution/src/hl_digital_twin_view.cpp) | High-level task execution visualization |
| [interaction_execution/src/wordle_view.cpp](interaction_execution/src/wordle_view.cpp) | Embedded Wordle web view bridge |
| [interaction_execution/launch/gui.launch.py](interaction_execution/launch/gui.launch.py) | Launch file for GUI/coordinator and optional perception, gamification, voice, and motion startup |

## 7.3 ROS 2 Interface

### Mission Coordinator Publishers

| **Topic** | **Type** | **Description** |
| --- | --- | --- |
| `/mission/state` | `std_msgs/String` | Perception/gamification state signal. Publishes `"SCANNING"` in SCANNING/RECOVERING and `"IDLE"` in other coordinator states. Uses transient local QoS. |
| `/wordle_bot/mission_state` | `std_msgs/String` | Current coordinator state, for example `IDLE`, `SCANNING`, `READY_TO_MOVE`, `MOVING`, `STOPPED`, `SAFETY_STOPPED`. Uses transient local QoS. |
| `/wordle_bot/mission_progress` | `std_msgs/String` | JSON progress summary used by the GUI diagnostics view. Uses transient local QoS. |
| `/wordle_bot/set_mission` | `geometry_msgs/PoseArray` | Configured task/home pose array for the motion subsystem. Current coordinator implementation publishes one pose from parameters. |
| `/wordle_bot/start_mission` | `std_msgs/Bool` | Start signal published after the coordinator sends a configured mission. |
| `/wordle_bot/stop_mission` | `std_msgs/Bool` | Stop signal published on STOP command, safety stop, motion timeout, or ABORT command. |
| `/wordle_bot/resume_mission` | `std_msgs/Bool` | Publisher exists in the coordinator, but the current coordinator command path does not publish it. The GUI publishes this topic directly. |
| `/wordle_bot/abort_mission` | `std_msgs/Bool` | Publisher exists in the coordinator, but the current coordinator command path does not publish it. The GUI publishes this topic directly. |

### Mission Coordinator Subscribers

| **Topic** | **Type** | **Description** |
| --- | --- | --- |
| `/wordle_bot/mission_cmd` | `std_msgs/String` | Operator command input. Supported strings are `START`, `STOP`, `RESUME`, `HOME`, and `ABORT`. |
| `/perception/human_detected` | `std_msgs/Bool` | Human detection flag. `true` triggers the safety guard. |
| `/perception/status` | `std_msgs/String` | Latest perception status, stored for progress reporting. |
| `/perception/detections` | `std_msgs/String` | JSON detections payload. The coordinator currently counts occurrences of `"letter"` to decide whether enough detections exist. |
| `/wordle_bot/mission_complete` | `std_msgs/Bool` | Motion subsystem completion signal used by the coordinator to leave MOVING/HOMING and return to IDLE. |

### GUI Publishers

| **Topic** | **Type** | **Description** |
| --- | --- | --- |
| `/wordle_bot/mission_cmd` | `std_msgs/String` | GUI START/STOP commands to the coordinator. |
| `/mission/state` | `std_msgs/String` | GUI scan/reset signal for perception/gamification. |
| `/wordle_bot/start_mission` | `std_msgs/Bool` | Direct start signal sent by the GUI when START is pressed. |
| `/wordle_bot/resume_mission` | `std_msgs/Bool` | Direct resume signal sent by the GUI when RESUME is pressed. |
| `/wordle_bot/stop_mission` | `std_msgs/Bool` | Direct stop signal sent by the GUI when STOP is pressed. |
| `/wordle_bot/scan_and_sweep` | `std_msgs/Bool` | Direct scan-and-sweep signal sent by the Scan Game Board button. |
| `/wordle_bot/abort_mission` | `std_msgs/Bool` | Direct abort signal sent by the GUI when the STOP button is contextually changed to ABORT. |
| `/wordle_bot/return_home` | `std_msgs/Bool` | Direct return-home signal sent by the Home button. |
| `/gamification/feedback` | `std_msgs/String` | Wordle feedback string such as `GBIBI`. |
| `/gamification/mode` | `std_msgs/String` | Publishes `MODE_A` or `MODE_B`. |
| `/gamification/secret_word` | `std_msgs/String` | Secret word for Mode A. |
| `/gamification/player_guess` | `std_msgs/String` | Player guess for Mode B. |

### GUI Subscribers

| **Topic** | **Type** | **Description** |
| --- | --- | --- |
| `/mission/state` | `std_msgs/String` | Perception state display and diagnostics. |
| `/wordle_bot/mission_state` | `std_msgs/String` | Coordinator state display and safety button state. |
| `/wordle_bot/robot_state` | `std_msgs/String` | Robot state display and scan completion handling. |
| `/wordle_bot/mission_complete` | `std_msgs/Bool` | GUI completion handling. |
| `/wordle_bot/mission_progress` | `std_msgs/String` | Mission progress JSON rendered in diagnostics. |
| `/perception/human_detected` | `std_msgs/Bool` | GUI safety banner and local STOP command trigger. |
| `/perception/status` | `std_msgs/String` | Perception diagnostics. |
| `/perception/detections` | `std_msgs/String` | Detection count for diagnostics. |
| `/gamification/guess` | `std_msgs/String` | Current robot guess shown on the Wordle board. |
| `/diagnostics` | `std_msgs/String` | Gamification diagnostics shown in the GUI. |

### Services

SS3 does not currently provide or call ROS services. Gamification owns `/gamification/reset` and `/gamification/undo`, but the current GUI reset path only publishes `/mission/state = "RESET"`.

## 7.4 Coordinator Structure

The coordinator uses a 500 ms wall timer and a priority-ordered tick function:

```text
tickTree()
+-- tickSafetyGuard()
+-- tickFailureDetectionBranch()
+-- tickRecoveryBranch()
+-- tickCommandBranch()
+-- tickMotionBranch()
`-- tickScanBranch()
```

The methods return `NodeStatus` values, but the implementation is an imperative state machine rather than a formal behavior tree library.

### Branch Behavior

| **Branch** | **Current behavior** |
| --- | --- |
| Safety guard | If `human_detected_` is true, clears pending command/goal state, publishes `/wordle_bot/stop_mission` once per safety event, transitions to `SAFETY_STOPPED`, and blocks lower-priority branches. |
| Failure detection | Checks perception timeout while SCANNING only when `perception_timeout_s > 0.0`; checks motion timeout while MOVING/HOMING and awaiting completion. |
| Recovery | Transitions `SAFETY_STOPPED -> STOPPED` after human detection clears; retries scan from `RECOVERING` while retries remain. |
| Command | Handles `START`, `STOP`, `RESUME`, `HOME`, and `ABORT` from `/wordle_bot/mission_cmd`. |
| Motion | Dispatches a configured task/home pose only when `pending_goal_request_ != NONE` and `auto_dispatch_motion` is true, or directly for HOME/ABORT coordinator commands. Waits for `/wordle_bot/mission_complete`. |
| Scan | In SCANNING, transitions to READY_TO_MOVE when `latest_detection_count_ >= minimum_detected_blocks`. |

### Fallback and Recovery Logic

| **Failure Scenario** | **Detection Method** | **Recovery Path** |
| --- | --- | --- |
| **Perception timeout** | `stateElapsedSeconds() >= perception_timeout_s` while SCANNING, with `perception_timeout_s > 0.0` and insufficient detections | `RECOVERING -> SCANNING` while `scan_retry_count_ < max_scan_retries`; then `PERCEPTION_FAILED` |
| **Motion timeout** | `motionElapsedSeconds() >= motion_timeout_s` while MOVING/HOMING and awaiting completion | Publish `/wordle_bot/stop_mission`, clear pending goal state, transition to `MOTION_FAILED` |
| **Human detection** | `/perception/human_detected = true` | Publish `/wordle_bot/stop_mission`, transition to `SAFETY_STOPPED`; after human detection clears, transition to `STOPPED` |
| **Insufficient detections** | Detection count below `minimum_detected_blocks` | Remain in SCANNING until detections arrive or an enabled perception timeout fires |

Scene change and unknown object topics are published by perception, but SS3 does not currently subscribe to or act on `/perception/scene_changed` or `/perception/out_of_category`.

## 7.5 GUI

### Layout and Views

The Qt GUI uses a drawer/content layout with:

| **View** | **Content** |
| --- | --- |
| **Sim** | Embedded RViz simulation view. |
| **Task** | High-level task/digital twin view. |
| **Camera** | Camera view with raw/computer-vision mode switching. |
| **Diagnostics** | Separate diagnostics window opened from the drawer; includes mission progress JSON, gamification diagnostics JSON, summary cards, and an event log. |
| **Wordle** | Embedded Wordle web UI integrated into the side panel. |

### Controls and Publications

| **Control** | **Action** | **Topic(s) Published** | **Payload** |
| --- | --- | --- | --- |
| Start | Begin coordinator flow and send direct motion start signal | `/wordle_bot/mission_cmd`, `/wordle_bot/start_mission` | `"START"`, `Bool(true)` |
| Resume | Resume motion when robot state indicates RUNNING/STOPPED | `/wordle_bot/mission_cmd`, `/wordle_bot/resume_mission` | `"RESUME"`, `Bool(true)` |
| Stop | Stop active/homing work | `/mission/state`, `/wordle_bot/mission_cmd`, `/wordle_bot/stop_mission` | `"IDLE"`, `"STOP"`, `Bool(true)` |
| Abort | Contextual STOP button action when coordinator is STOPPED | `/wordle_bot/abort_mission` | `Bool(true)` |
| Scan Game Board | Trigger perception scan and motion scan sweep | `/mission/state`, `/wordle_bot/scan_and_sweep` | `"SCANNING"`, `Bool(true)` |
| Home | Return robot to home/working pose through motion subsystem | `/wordle_bot/return_home` | `Bool(true)` |
| Record | Start/stop voice helper recording | Voice helper subprocess stdin | Helper command |
| Confirm | Confirm a voice/player guess result | `/gamification/player_guess` | Uppercase five-letter guess |
| Feedback row | Submit manual Wordle feedback | `/gamification/feedback` | Feedback string such as `GBIBI` |
| Mode selector | Set game mode | `/gamification/mode` | `MODE_A` or `MODE_B` |
| Secret word | Set Mode A secret word | `/gamification/secret_word` | Uppercase five-letter word |

## 7.6 Safety Monitor

The coordinator safety guard executes first on every 500 ms heartbeat.

```cpp
MissionCoordinator::NodeStatus MissionCoordinator::tickSafetyGuard()
{
  if (!human_detected_) {
    return NodeStatus::FAILURE;
  }

  pending_command_.clear();
  awaiting_motion_completion_ = false;
  motion_goal_sent_ = false;
  pending_goal_request_ = GoalRequest::NONE;
  if (!safety_stop_published_) {
    publishMissionSignal(stop_mission_pub_, kStopMissionTopic);
    safety_stop_published_ = true;
  }
  transitionTo(MissionState::SAFETY_STOPPED, "Human detected by perception");
  return NodeStatus::RUNNING;
}
```

### Trigger Conditions

| **Trigger** | **Source Topic** | **Condition** | **Action** |
| --- | --- | --- | --- |
| **Human detected** | `/perception/human_detected` | `data == true` | Coordinator publishes `/wordle_bot/stop_mission`, clears pending goal/command state, transitions to `SAFETY_STOPPED`. GUI also publishes `/mission/state = "IDLE"` and `/wordle_bot/mission_cmd = "STOP"` when it first sees the event. |
| **Safety clear** | `/perception/human_detected` | `data == false` while coordinator is `SAFETY_STOPPED` | Coordinator transitions to `STOPPED`; operator must choose RESUME/START/HOME depending on GUI state. |

There is no implemented `AUTO_RESUME_AFTER_SAFETY` parameter. The current behavior after safety clears is STOPPED, not automatic resume.

## 7.7 Mission Control & State Machine

### State Diagram

```text
IDLE
  | START command
  v
SCANNING
  | enough detections
  v
READY_TO_MOVE
  | auto_dispatch_motion=true
  v
MOVING
  | /wordle_bot/mission_complete=true
  v
IDLE

SCANNING -- timeout, retries remain --> RECOVERING --> SCANNING
SCANNING -- timeout, retries exhausted --> PERCEPTION_FAILED

HOME command or ABORT command while safe --> HOMING
HOMING -- /wordle_bot/mission_complete=true --> IDLE
MOVING/HOMING -- motion timeout --> MOTION_FAILED

Any state with /perception/human_detected=true --> SAFETY_STOPPED
SAFETY_STOPPED -- human_detected=false --> STOPPED
STOPPED -- RESUME --> READY_TO_MOVE if detections exist, otherwise SCANNING
```

### States and Transitions

| **State** | **Entry Condition** | **Exit Condition** | **Coordinator Publications** |
| --- | --- | --- | --- |
| **IDLE** | Initial state; mission completion | START/HOME/ABORT/safety event | `/mission/state = "IDLE"`, `/wordle_bot/mission_state = "IDLE"` |
| **SCANNING** | START/RESUME command, or recovery retry | Enough detections, timeout, STOP, safety event | `/mission/state = "SCANNING"` |
| **READY_TO_MOVE** | Enough detections while SCANNING | Auto dispatch, STOP, RESUME, safety event | `/mission/state = "IDLE"` |
| **MOVING** | Configured task pose dispatched | `/wordle_bot/mission_complete`, motion timeout, STOP, safety event | `/wordle_bot/set_mission`, `/wordle_bot/start_mission`, `/mission/state = "IDLE"` |
| **STOPPED** | STOP command or safety clear | START/RESUME/HOME/ABORT/safety event | `/wordle_bot/stop_mission` on STOP entry |
| **SAFETY_STOPPED** | Human detected | Human detection clears | `/wordle_bot/stop_mission`, `/mission/state = "IDLE"` |
| **RECOVERING** | Perception timeout with retries remaining | Immediate retry or retries exhausted | `/mission/state = "SCANNING"` |
| **HOMING** | HOME command or ABORT command while safe | `/wordle_bot/mission_complete`, motion timeout, safety event | `/wordle_bot/set_mission`, `/wordle_bot/start_mission` |
| **PERCEPTION_FAILED** | Perception timeout after retries exhausted | Manual command | `/mission/state = "IDLE"` |
| **MOTION_FAILED** | Motion timeout while MOVING/HOMING | Manual command | `/wordle_bot/stop_mission`, `/mission/state = "IDLE"` |
| **ERROR** | Defined in enum but not explicitly entered by current code | N/A | `/mission/state = "IDLE"` if entered |

## 7.8 Configurable Parameters

These are declared by `mission_coordinator_node`. `interaction_execution/launch/gui.launch.py` exposes the main grading/demo parameters (`auto_dispatch_motion`, `minimum_detected_blocks`, `perception_timeout_s`, `max_scan_retries`, and `motion_timeout_s`) as launch arguments.

| **Parameter** | **Default** | **Description** |
| --- | --- | --- |
| `auto_dispatch_motion` | `false` | If true, READY_TO_MOVE dispatches the configured task pose and transitions to MOVING. If false, READY_TO_MOVE holds without coordinator dispatch. |
| `minimum_detected_blocks` | `1` | Minimum detected letter count required for scan completion. |
| `perception_timeout_s` | `0.0` | Perception timeout in seconds. `0.0` disables perception timeout recovery. |
| `max_scan_retries` | `1` | Number of automatic scan retries after perception timeout. |
| `motion_timeout_s` | `20.0` | Seconds to wait for `/wordle_bot/mission_complete` while MOVING/HOMING before MOTION_FAILED. |
| `mission_signal_publish_count` | `5` | Number of times to publish mission Bool signals. |
| `mission_signal_publish_period_ms` | `100` | Delay between repeated mission Bool signal publications. |
| `mission_signal_wait_for_subscriber_s` | `1.0` | Maximum wait for a matched subscriber before publishing a mission signal. |
| `goal_frame_id` | `ur_base_link` | Frame for configured task pose. |
| `home_frame_id` | `ur_base_link` | Frame for configured home pose. |
| `goal.x`, `goal.y`, `goal.z` | `0.30`, `0.25`, `0.25` | Configured task pose position. |
| `goal.roll`, `goal.pitch`, `goal.yaw` | `pi`, `0.0`, `0.0` | Configured task pose orientation, converted from RPY to quaternion. |
| `home.x`, `home.y`, `home.z` | `0.30`, `0.00`, `0.30` | Configured home pose position. |
| `home.roll`, `home.pitch`, `home.yaw` | `pi`, `0.0`, `0.0` | Configured home pose orientation, converted from RPY to quaternion. |

## 7.9 SS3 Limitations

- **No formal behavior tree library:** The coordinator is a behavior-tree-style state machine implemented imperatively.
- **Fixed tick rate:** The coordinator heartbeat is hardcoded to 500 ms.
- **Perception timeout disabled by default:** `perception_timeout_s` defaults to `0.0`, so timeout/retry behavior must be enabled with a parameter.
- **Coordinator does not parse `/gamification/mission_state`:** Gamification publishes ordered placement JSON, but the SS3 coordinator currently dispatches configured parameter poses only.
- **Coordinator does not act on scene-change or unknown-object topics:** Perception publishes `/perception/scene_changed` and `/perception/out_of_category`, but SS3 does not subscribe to them.
- **GUI and coordinator both publish some control signals:** The GUI publishes direct motion signals as well as coordinator commands, so the exact command path depends on which button is used.
- **No multi-user/role model:** The GUI assumes a single operator.
- **No persistent coordinator state:** Mission/coordinator progress is not restored after node restart.
- **No ROS service integration in SS3:** GUI reset does not call gamification reset/undo services.
- **ROS domain and middleware dependent:** Cross-machine operation requires matching ROS domain ID and compatible RMW configuration.

## 7.10 Showcase Test Guide - Contract Criteria Walkthrough

These tests focus on behavior implemented in the current code.

### Prerequisites

**Terminal 1 - Mission Coordinator + GUI:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false
```

**Terminal 2 - Monitor coordinator state:**
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
```

**Optional monitors:**
```bash
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local
ros2 topic echo /wordle_bot/stop_mission
ros2 topic echo /wordle_bot/set_mission
ros2 topic echo /wordle_bot/start_mission
```

### Criterion P - Basic GUI and Coordinator Commands

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| 1 | Launch the GUI/coordinator. | GUI renders and `/wordle_bot/mission_state` starts at `IDLE`. |
| 2 | Publish `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"`. | GUI/coordinator are clear to accept operator actions. |
| 3 | Press START. | Coordinator receives `START` and publishes `SCANNING`. |
| 4 | Press STOP while active. | Coordinator receives `STOP`, publishes `/wordle_bot/stop_mission`, and transitions to `STOPPED`. |

**PASS:** The GUI can drive basic coordinator state changes and the state topic reflects those changes.

### Criterion C - Safety Gating

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| 1 | Ensure `/perception/human_detected=false`. | System accepts commands. |
| 2 | Press START. | `/wordle_bot/mission_state = SCANNING`. |
| 3 | Publish `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"`. | Safety guard runs on the next tick. |
| 4 | Monitor state and stop topic. | `/wordle_bot/mission_state = SAFETY_STOPPED`; `/wordle_bot/stop_mission` publishes `true`. |
| 5 | Publish `ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"`. | Coordinator transitions to `STOPPED`, not automatic resume. |

**PASS:** Human detection has priority over lower coordinator branches and prevents continued scan/motion behavior.

### Criterion D1 - Detection-Conditioned Transition

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| 1 | Ensure `/perception/human_detected=false`. | Safe state. |
| 2 | Press START or publish `START` to `/wordle_bot/mission_cmd`. | State becomes `SCANNING`. |
| 3 | Publish `ros2 topic pub --once /perception/detections std_msgs/msg/String "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"`. | Detection count becomes 1. |
| 4 | Check `/wordle_bot/mission_state`. | State becomes `READY_TO_MOVE` because `minimum_detected_blocks` defaults to 1. |

**PASS:** The transition depends on `hasEnoughDetections()`, not a fixed delay.

### Criterion D2 - Perception Timeout Recovery

For this test, enable the coordinator timeout parameters through the launch file:

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false \
  perception_timeout_s:=5.0 \
  max_scan_retries:=1
```

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| 1 | Publish `/perception/human_detected=false`. | Safe state. |
| 2 | Publish `START` to `/wordle_bot/mission_cmd`. | State becomes `SCANNING`. |
| 3 | Do not publish detections. | Scan remains below the minimum. |
| 4 | Wait past the timeout. | State transitions `RECOVERING -> SCANNING`. |
| 5 | Wait through the second timeout without detections. | State transitions to `PERCEPTION_FAILED`. |

**PASS:** Timeout/retry behavior works when `perception_timeout_s` is explicitly enabled.

### Criterion D3 - Motion Dispatch and Completion

This test requires `auto_dispatch_motion=true` and either a real/sim motion subsystem or a manual completion publication.

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false \
  auto_dispatch_motion:=true
```

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| 1 | Publish `/perception/human_detected=false`. | Safe state. |
| 2 | Publish `START` and then one valid detection. | State becomes `READY_TO_MOVE`, then `MOVING`. |
| 3 | Monitor `/wordle_bot/set_mission` and `/wordle_bot/start_mission`. | Coordinator publishes a one-pose `PoseArray` and `Bool(true)`. |
| 4 | Publish `ros2 topic pub --once /wordle_bot/mission_complete std_msgs/msg/Bool "{data: true}"`. | Coordinator transitions back to `IDLE`. |

**PASS:** READY_TO_MOVE dispatches a configured goal only when `auto_dispatch_motion=true`, and completion is driven by `/wordle_bot/mission_complete`.

### Criterion HD - Priority and Interruption During Motion

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| 1 | Reach `MOVING` using D3. | Coordinator is awaiting `/wordle_bot/mission_complete`. |
| 2 | Publish `/perception/human_detected=true`. | Safety guard runs before motion completion handling. |
| 3 | Check state and stop signal. | State becomes `SAFETY_STOPPED`; `/wordle_bot/stop_mission` publishes `true`; pending motion state is cleared. |
| 4 | Publish `/perception/human_detected=false`. | State becomes `STOPPED`. |

**PASS:** Safety priority is independent of motion state and interrupts the active coordinator branch.

### Quick Diagnostics Table

| **Symptom** | **Likely Cause** | **Fix** |
| --- | --- | --- |
| GUI does not start | Qt display problem or missing Qt dependency | Check `QT_QPA_PLATFORM=xcb` and display/X11 forwarding. |
| START is blocked | GUI currently sees `human_detected=true` or safety mode is not idle/stopped | Publish `/perception/human_detected=false` and wait for controls to update. |
| SCANNING never reaches READY_TO_MOVE | No detection payload or detection count below `minimum_detected_blocks` | Publish a valid `/perception/detections` payload containing at least one `"letter"`. |
| Perception timeout never fires | `perception_timeout_s` defaults to `0.0` | Launch with `perception_timeout_s:=5.0` or another positive value. |
| READY_TO_MOVE never dispatches motion | `auto_dispatch_motion` defaults to `false` | Launch with `auto_dispatch_motion:=true`. |
| Coordinator does not leave MOVING | It waits for `/wordle_bot/mission_complete`, not `/wordle_bot/motion_complete` | Ensure the motion subsystem publishes `/wordle_bot/mission_complete=true` or publish it manually for a test. |
| Scan Game Board does not change coordinator state as expected | The GUI Scan Game Board button publishes `/mission/state` and `/wordle_bot/scan_and_sweep`, not `/wordle_bot/mission_cmd START` | Use START for coordinator SCANNING tests; use Scan Game Board for direct motion scan-sweep behavior. |
