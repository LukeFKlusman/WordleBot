# Interaction Execution (SS3) Grading Test Cases

These test cases map SS3 evidence to the P-HD grading rubric using the behavior implemented in the current code. The coordinator and GUI can be tested without the full system for basic state, safety, and detection-condition behavior. Motion dispatch tests require either manual topic injection or the motion subsystem.

---

## Common Setup

**Build and source the workspace:**
```bash
cd ~/ros2_ws
colcon build --packages-select interaction_execution
source install/setup.bash
```

**Launch GUI + coordinator only:**
```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false
```

**Common monitor terminals:**
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local
ros2 topic echo /wordle_bot/stop_mission
ros2 topic echo /wordle_bot/set_mission
ros2 topic echo /wordle_bot/start_mission
```

**Useful input topics:**
```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
```

---

# PASS Level - Basic GUI Functionality

## TC-P1: GUI Launches and Responds to START/STOP

**Criterion:** The GUI renders and basic coordinator commands work.

**Steps:**

1. Launch GUI + coordinator using Common Setup.
2. Confirm the Qt window renders with drawer buttons, the default Sim/RViz view, Wordle panel, voice controls, and safety controls.
3. Publish safe human state:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   ```
4. Press START in the safety panel.
5. Observe `/wordle_bot/mission_state`.
6. Press STOP while the GUI is active.
7. Observe `/wordle_bot/mission_state` and `/wordle_bot/stop_mission`.

**Expected Result:**

- GUI window appears without crashes.
- START publishes a coordinator command and state becomes `SCANNING`.
- STOP publishes a coordinator command and `/wordle_bot/stop_mission`; state becomes `STOPPED`.
- No exceptions appear in terminal output.

**Evidence to Record:**

- Screenshot of the GUI with safety controls visible.
- Terminal output showing `SCANNING -> STOPPED`.
- Terminal output showing `/wordle_bot/stop_mission` with `data: true`.

**PASS:** The GUI successfully launches and can drive basic coordinator state changes.

---

# CREDIT Level - Safe and Consistent Control

## TC-C1: Safety Interrupt Produces a Valid Recovery State

**Criterion:** Human detection interrupts active coordinator work and clears to a deterministic recovery state.

**Steps:**

1. Publish safe human state:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   ```
2. Press START or publish:
   ```bash
   ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'START'}"
   ```
3. Confirm `/wordle_bot/mission_state = SCANNING`.
4. Publish human detection:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
5. Check state within one second.
6. Publish human clear:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   ```
7. Check state again.

**Expected Result:**

- State changes from `SCANNING` to `SAFETY_STOPPED`.
- `/wordle_bot/stop_mission` publishes `data: true`.
- After human detection clears, state changes to `STOPPED`.
- The coordinator does not automatically resume scanning or motion.

**PASS:** Safety has priority and recovery is explicit.

---

## TC-C2: START Is Blocked While Human Detection Is Active

**Criterion:** Safety is evaluated before command processing.

**Steps:**

1. Publish human detection:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
2. Press START or publish:
   ```bash
   ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'START'}"
   ```
3. Check `/wordle_bot/mission_state`.
4. Monitor `/wordle_bot/start_mission`.

**Expected Result:**

- State is `SAFETY_STOPPED`.
- No configured motion dispatch occurs.
- `/wordle_bot/start_mission` is not sent by the coordinator path.

**PASS:** The coordinator does not enter a running task state while unsafe.

---

# DISTINCTION Level - Behavior-Tree-Style Coordinator

## TC-D1: Condition-Based Scan to Ready Transition

**Criterion:** The coordinator evaluates `hasEnoughDetections()` before transitioning from SCANNING to READY_TO_MOVE.

**Steps:**

1. Publish safe human state.
2. Press START or publish `START` to `/wordle_bot/mission_cmd`.
3. Confirm state is `SCANNING`.
4. Publish one valid detection:
   ```bash
   ros2 topic pub --once /perception/detections std_msgs/msg/String \
     "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
   ```
5. Check `/wordle_bot/mission_state`.

**Expected Result:**

- State becomes `READY_TO_MOVE`.
- The transition occurs because the detection count is at least `minimum_detected_blocks`, which defaults to `1`.

**PASS:** The scan branch is condition-driven rather than time-driven.

---

## TC-D2: Parallel Safety Monitor Interrupts Active Scan

**Criterion:** The safety guard runs before scan/motion branches on every 500 ms tick.

**Steps:**

1. Monitor `/wordle_bot/stop_mission`.
2. Publish safe human state.
3. Start scanning.
4. While state is `SCANNING`, publish:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
5. Check state and stop signal.

**Expected Result:**

- State becomes `SAFETY_STOPPED`.
- `/wordle_bot/stop_mission` publishes `data: true`.
- The scan branch no longer progresses until safety clears.

**PASS:** Safety monitoring has highest priority.

---

## TC-D3: Perception Timeout and Retry Recovery

**Criterion:** The coordinator can detect scan timeout and retry when timeout is enabled.

`perception_timeout_s` defaults to `0.0`, so timeout recovery is disabled in the normal launch configuration. Enable it through launch arguments for this test:

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false \
  perception_timeout_s:=5.0 \
  max_scan_retries:=1
```

In a second terminal:
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local
```

**Steps:**

1. Publish safe human state.
2. Publish:
   ```bash
   ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'START'}"
   ```
3. Do not publish detections.
4. Wait longer than 5 seconds.
5. Observe `RECOVERING -> SCANNING`.
6. Do not publish detections during the retry.
7. Wait longer than 5 seconds again.

**Expected Result:**

- First timeout transitions `SCANNING -> RECOVERING -> SCANNING`.
- Second timeout transitions to `PERCEPTION_FAILED`.
- `/wordle_bot/mission_progress` explains the active recovery/failure state.

**PASS:** Timeout recovery works when the parameter is explicitly enabled.

---

## TC-D4: Motion Dispatch Requires Auto Dispatch

**Criterion:** READY_TO_MOVE only dispatches a coordinator-configured pose when `auto_dispatch_motion=true`.

Enable auto dispatch through the launch file for this test:

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false \
  auto_dispatch_motion:=true
```

Monitor:
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /wordle_bot/set_mission
ros2 topic echo /wordle_bot/start_mission
```

**Steps:**

1. Publish safe human state.
2. Publish `START` to `/wordle_bot/mission_cmd`.
3. Publish one valid detection.
4. Watch `/wordle_bot/set_mission` and `/wordle_bot/start_mission`.
5. Simulate completion:
   ```bash
   ros2 topic pub --once /wordle_bot/mission_complete std_msgs/msg/Bool "{data: true}"
   ```

**Expected Result:**

- State transitions `SCANNING -> READY_TO_MOVE -> MOVING`.
- Coordinator publishes one configured pose on `/wordle_bot/set_mission`.
- Coordinator publishes `Bool(true)` on `/wordle_bot/start_mission`.
- `/wordle_bot/mission_complete=true` returns the coordinator to `IDLE`.

**PASS:** Motion dispatch is parameter-gated and completion uses `/wordle_bot/mission_complete`.

---

# HIGH DISTINCTION Level - Integrated Priority and Recovery Behavior

## TC-HD1: Safety Interrupt During Motion

**Criterion:** Safety monitoring interrupts even while the coordinator is awaiting mission completion.

**Setup:** Run TC-D4 until the coordinator reaches `MOVING`, but do not publish mission completion yet.

**Steps:**

1. Confirm `/wordle_bot/mission_state = MOVING`.
2. Publish:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
3. Monitor state and `/wordle_bot/stop_mission`.
4. Publish human clear:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   ```

**Expected Result:**

- State changes `MOVING -> SAFETY_STOPPED`.
- `/wordle_bot/stop_mission` publishes `data: true`.
- Pending motion state is cleared by the safety guard.
- After human clear, state becomes `STOPPED`.

**PASS:** Safety priority is independent of the active coordinator branch.

---

## TC-HD2: Autonomous Recovery From Transient Perception Timeout

**Criterion:** A transient scan failure can recover without an additional operator command.

Enable timeout recovery through launch arguments:

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false \
  perception_timeout_s:=3.0 \
  max_scan_retries:=2
```

**Steps:**

1. Publish safe human state.
2. Publish `START` to `/wordle_bot/mission_cmd`.
3. Wait until state reaches `RECOVERING`, then `SCANNING` again.
4. During the retry SCANNING state, publish one valid detection.
5. Check `/wordle_bot/mission_state`.

**Expected Result:**

```text
SCANNING
RECOVERING
SCANNING
READY_TO_MOVE
```

No additional START/RESUME command is required after the retry begins.

**PASS:** The recovery branch can autonomously retry and rejoin the normal scan path.

---

## TC-HD3: Dynamic Branch Switching Across Scan, Motion, Failure, and Recovery

**Criterion:** The active coordinator behavior changes based on state and conditions.

Enable auto dispatch and a short motion timeout through launch arguments:

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false \
  auto_dispatch_motion:=true \
  motion_timeout_s:=5.0
```

**Steps:**

1. Publish safe human state.
2. Publish `START`.
3. Publish one valid detection.
4. Confirm state reaches `MOVING`.
5. Do not publish `/wordle_bot/mission_complete`.
6. Wait longer than 5 seconds.
7. Publish a coordinator HOME command:
   ```bash
   ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'HOME'}"
   ```

**Expected Sequence:**

```text
SCANNING
READY_TO_MOVE
MOVING
MOTION_FAILED
HOMING
```

If no motion subsystem is running, HOMING will also wait for `/wordle_bot/mission_complete` or eventually time out based on `motion_timeout_s`.

**PASS:** The coordinator switches between scan, motion, failure detection, command, and motion-recovery behavior using tick priority and state.

---

# Evidence Summary and Rubric Mapping

| **Grade** | **Criteria** | **Test Cases** | **Evidence** |
| --- | --- | --- | --- |
| **P** | GUI launches; basic START/STOP commands work | TC-P1 | Screenshot plus `/wordle_bot/mission_state` and `/wordle_bot/stop_mission` output |
| **C** | Safe, consistent state transitions | TC-C1, TC-C2 | `SCANNING -> SAFETY_STOPPED -> STOPPED`; blocked START while unsafe |
| **D** | Behavior-tree-style ticks, condition-based scan, safety priority, timeout recovery, gated motion dispatch | TC-D1, TC-D2, TC-D3, TC-D4 | State logs, stop signal, mission progress JSON, set/start mission topics |
| **HD** | Safety interruption during motion, autonomous retry recovery, dynamic branch switching | TC-HD1, TC-HD2, TC-HD3 | State sequences showing branch changes and recovery without extra operator input |

---

# Quick Reference: Topic Publishing Cheatsheet

**Set human detection:**
```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
```

**Send coordinator commands:**
```bash
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'START'}"
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'STOP'}"
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'RESUME'}"
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'HOME'}"
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: 'ABORT'}"
```

**Publish one detection:**
```bash
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
```

**Simulate coordinator motion completion:**
```bash
ros2 topic pub --once /wordle_bot/mission_complete std_msgs/msg/Bool "{data: true}"
```

**Monitor transient-local coordinator topics:**
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local
```

**Monitor live signal topics:**
```bash
ros2 topic echo /wordle_bot/stop_mission
ros2 topic echo /wordle_bot/set_mission
ros2 topic echo /wordle_bot/start_mission
```
