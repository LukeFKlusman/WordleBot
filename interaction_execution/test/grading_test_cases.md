# Interaction Execution (SS3) Grading Test Cases

These test cases map directly to the P–HD grading rubric and demonstrate the behavior tree mission control, safety monitoring, and state machine implementation. Tests are designed to run with just the coordinator and GUI (no full system required for basic demonstrations).

---

## Common Setup

**Build and source the workspace:**
```bash
cd ~/ros2_ws
colcon build --packages-select interaction_execution
source install/setup.bash
```

**Core launch (GUI + Coordinator only):**
```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false \
  launch_perception:=false \
  launch_gamification:=false
```

**Monitoring utilities** (run in separate terminals as needed):
```bash
# Mission state changes
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local

# Mission progress (JSON with step details)
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local

# Safety-related commands
ros2 topic echo /wordle_bot/stop_mission
ros2 topic echo /wordle_bot/start_mission

# Perception input
ros2 topic echo /perception/detections
ros2 topic echo /perception/human_detected
```

---

# PASS Level — Basic GUI Functionality

## **TC-P1: GUI Launches and Responds to START/STOP**

**Criterion:** The GUI renders without errors and basic command buttons work.

**Steps:**

1. Launch the GUI (as per Common Setup above)
2. Observe the Qt window renders with:
   - Left drawer (collapsed)
   - Central RViz view
   - Right panel: Wordle display + Voice controls + Safety controls
3. Press the blue "START" button in the safety panel
4. Observe `/wordle_bot/mission_state` in monitoring terminal
5. Press the red "STOP" button
6. Observe `/wordle_bot/mission_state` again

**Expected Result:**

- GUI window appears without crashes
- Safety controls panel is visible with START (blue), STOP (red), SCAN, and HOME buttons
- START button press → `/wordle_bot/mission_state` outputs `SCANNING`
- STOP button press → `/wordle_bot/mission_state` outputs `STOPPED`
- No exceptions in terminal output

**Evidence to Record:**

- Screenshot of GUI with safety panel visible
- Terminal output showing state transitions: `SCANNING` → `STOPPED`

**PASS:** The GUI successfully launches and sends commands that the coordinator processes.

---

# CREDIT Level — GUI Control with No Unintended Behaviors

## **TC-C1: State Machine Prevents Invalid Transitions**

**Criterion:** The coordinator maintains a valid state machine; unsafe states cannot be entered, and transitions follow defined rules.

**Setup:**

Run the coordinator with perception simulation:
```bash
# Terminal 1: Ensure human_detected starts as false
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"

# Terminal 2: Monitor state
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local

# Terminal 3: Launch GUI + Coordinator
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false launch_perception:=false launch_gamification:=false
```

**Steps:**

1. Press START → state should be `SCANNING`
2. While in SCANNING, publish human detection:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
3. **Immediately** check state (within 1 second; coordinator ticks every 500ms):
   - **Expected:** `SAFETY_STOPPED` (not still `SCANNING`, not `MOVING`)
4. Publish human cleared:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   ```
5. Check state again:
   - **Expected:** `STOPPED` (not auto-resume; human left but operator hasn't resumed)
6. Press HOME button:
   - **Expected:** `HOMING` (transition from STOPPED is valid)

**Why This Demonstrates C-Level:**

- **Safety priority:** The coordinator does not ignore human detection just because a scan is in progress
- **Explicit state validation:** SAFETY_STOPPED blocks invalid transitions; no undefined state jumps
- **No race conditions:** The 500ms heartbeat ensures state checks run regularly and consistently
- **Operator control:** User must explicitly press RESUME or HOME to recover from STOPPED

**PASS:** The state machine is a valid directed graph; unsafe combinations cannot occur, and transitions are deterministic.

---

## **TC-C2: Perception Safety Block Prevents START When Human Detected**

**Criterion:** Safety checks execute before command processing; START fails if unsafe.

**Steps:**

1. Set human detection to true:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
2. Press START button in GUI
3. Check mission state immediately:
   ```bash
   ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
   ```

**Expected Result:**

- State remains `IDLE` (or `SAFETY_STOPPED` if coordinator was already running)
- START command is effectively blocked by safety condition
- Motion is not dispatched
- No false START in logs

**PASS:** Safety conditions are checked before allowing state transitions; the coordinator respects safety invariants.

---

# DISTINCTION Level — Behavior Tree Implementation

## **TC-D1: Condition-Based State Transition (Scan → Ready)**

**Criterion:** The coordinator evaluates a condition (`hasEnoughDetections()`) before transitioning from SCANNING to READY_TO_MOVE.

**Setup:**

Run with `minimum_detected_blocks:=1` (default):
```bash
# Terminal 1: Monitor state
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local

# Terminal 2: Launch coordinator
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false launch_perception:=false launch_gamification:=false
```

**Steps:**

1. Set human_detected = false:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   ```
2. Press START button → state = `SCANNING`
3. Publish **one** detection with valid JSON:
   ```bash
   ros2 topic pub --once /perception/detections std_msgs/msg/String \
     "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
   ```
4. Wait up to 1 second and check state:
   ```bash
   ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
   ```

**Expected Result:**

- State transitions to `READY_TO_MOVE`
- NOT an immediate jump; the coordinator polls the condition every heartbeat
- Logs show: `SCANNING` → evaluated `hasEnoughDetections()` → true → `READY_TO_MOVE`

**Why This Demonstrates D-Level:**

- The transition is **condition-based**, not hard-coded
- The coordinator implements a behavior tree tick loop (heartbeat)
- The `hasEnoughDetections()` check is a **selector node** in the scan branch
- If detections were insufficient (count < minimum), the state would remain SCANNING

**PASS:** The coordinator does not jump states blindly; it evaluates a semantic condition before transitioning.

---

## **TC-D2: Parallel Safety Monitor Interrupts Active Task**

**Criterion:** Safety monitoring runs in parallel with other branches; human detection immediately halts motion (or scan).

**Setup:**

```bash
# Terminal 1: Monitor stop_mission signal (for evidence of safety interrupt)
ros2 topic echo /wordle_bot/stop_mission

# Terminal 2: Monitor state
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local

# Terminal 3: Launch coordinator
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false launch_perception:=false launch_gamification:=false
```

**Steps:**

1. Set human_detected = false, press START:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   # (press START in GUI)
   # State should be SCANNING
   ```
2. **While SCANNING**, publish human detection:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
3. **Within 1 second**, check state:
   - Should be `SAFETY_STOPPED` (not `SCANNING`)
4. In Terminal 1, verify `/wordle_bot/stop_mission` was published:
   ```bash
   # Terminal 1 should show: data: true
   ```

**Why This Demonstrates D-Level:**

- **Parallel monitoring:** Safety branch ticks every 500ms, independent of scan state
- **Interruption priority:** Safety condition blocks subsequent task branches
- **Immediate propagation:** Stop signal sent to motion subsystem without delay
- **Multiple concurrent monitors:** The coordinator ticks safety, failure detection, recovery, command, scan, and motion branches in parallel

**PASS:** Safety monitoring is a dedicated behavior tree branch with highest priority; human detection immediately halts any activity.

---

## **TC-D3: Failure Detection and Retry Recovery**

**Criterion:** The coordinator detects perception timeout, retries automatically (if configured), and escalates to failure state when retries exhausted.

**Setup:**

Launch with short timeout for testing:
```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false launch_perception:=false launch_gamification:=false \
  -- -p perception_timeout_s:=5.0 -p max_scan_retries:=1
```

Monitor progress (to see retry counter):
```bash
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local
```

**Steps:**

1. Set human_detected = false, press START:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   # State = SCANNING
   ```
2. **Do NOT publish any detections** (simulate perception failure)
3. Wait 6 seconds (timeout threshold is 5s)
4. Check mission state:
   ```bash
   ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
   ```
5. Optionally publish detections during retry:
   ```bash
   # If you want to show successful retry recovery:
   ros2 topic pub --once /perception/detections std_msgs/msg/String \
     "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
   ```
6. Check state again:
   - If detections published during retry: state → `READY_TO_MOVE`
   - If no detections during retry: state → `PERCEPTION_FAILED` (after max retries exhausted)

**Expected Sequence:**

- `SCANNING` (waiting for detections) → timeout
- `RECOVERING` (retry in progress) → detection arrives → `READY_TO_MOVE` (success)
- OR `RECOVERING` → timeout again → `PERCEPTION_FAILED` (max retries exhausted)

**Why This Demonstrates D-Level:**

- **Automatic failure detection:** No manual intervention; the coordinator polls elapsed time
- **Condition-based recovery:** Retry counter is checked; recovery only happens if retries remain
- **Escalation:** After max retries, the coordinator escalates to PERCEPTION_FAILED state
- **JSON progress:** Mission progress includes retry count and failure reason

**PASS:** The coordinator implements timeout-based failure detection with automatic retry logic.

---

## **TC-D4: Task Priority Selector (Safety > Scan > Motion)**

**Criterion:** The behavior tree evaluates branch conditions in priority order; safety must be checked before task execution.

**Setup:**

```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /wordle_bot/start_mission
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false launch_perception:=false launch_gamification:=false
```

**Steps:**

1. Set human_detected = false, press START → state = `SCANNING`
2. Publish detections to reach READY_TO_MOVE:
   ```bash
   ros2 topic pub --once /perception/detections std_msgs/msg/String \
     "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
   # State = READY_TO_MOVE
   ```
3. **Before motion dispatch**, trigger safety condition:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
4. Check state immediately:
   - Should be `SAFETY_STOPPED` (not `MOVING`)
5. In the `/wordle_bot/start_mission` monitor, verify NO signal was sent:
   - Motion was blocked by safety condition

**Why This Demonstrates D-Level:**

- **Priority selector:** The tree evaluates `tickSafetyGuard()` before `tickMotionBranch()`
- **Context-aware decisions:** Task branches skip if safety condition active
- **No race conditions:** The coordinator's heartbeat ensures consistent priority

**PASS:** Motion is blocked by safety conditions; safety has higher priority than task execution.

---

# HIGH DISTINCTION Level — Advanced Features

## **TC-HD1: Continuous Monitoring While Motion Active**

**Criterion:** Multiple branches tick simultaneously; safety and perception updates occur independently of motion state.

**Requirements:** This test requires the motion subsystem (SS2) to be running.

**Setup:**

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=true launch_perception:=false launch_gamification:=false \
  -- -p auto_dispatch_motion:=true
```

Monitor:
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local
ros2 topic echo /perception/human_detected
```

**Steps:**

1. Reach MOVING state (scan with detections, auto dispatch):
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
   # Press START
   ros2 topic pub --once /perception/detections std_msgs/msg/String \
     "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
   # Wait for MOVING state
   ```
2. **While MOVING**, publish updated detections:
   ```bash
   ros2 topic pub --once /perception/detections std_msgs/msg/String \
     "{data: '{\"blocks\":[{\"letter\":\"B\",\"x\":0.31,\"y\":0.11,\"z\":0.02}]}'}"
   ```
3. Check `/wordle_bot/mission_progress`:
   - Should include updated detection info (not stale from step 1)
4. **While still MOVING**, trigger human detection:
   ```bash
   ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
   ```
5. Check state immediately:
   - Should be `SAFETY_STOPPED` (motion interrupted)

**Expected Behavior:**

- Detections update in real-time, reflected in mission progress
- Safety condition interrupts even while motion active
- No blocking between branches; all tick every 500ms

**Why This Demonstrates HD-Level:**

- **Truly parallel branches:** Not sequential "if safety then else motion"
- **Non-blocking perception:** Detection updates don't stall motion
- **Context-agnostic safety:** Human detection works regardless of active branch
- **Sophisticated orchestration:** Multiple data sources and tasks coordinated simultaneously

**SEMI-PASS or PASS:** If motion subsystem available and auto-dispatch working, this demonstrates full parallel behavior tree execution. (Limited to PASS if motion not testable, as other HD criteria may still be verified.)

---

## **TC-HD2: Sophisticated Auto-Recovery Without Operator Intervention**

**Criterion:** The recovery branch can autonomously handle transient failures (e.g., perception timeout with immediate retry success) without manual operator action.

**Setup:**

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=false launch_perception:=false launch_gamification:=false \
  -- -p perception_timeout_s:=3.0 -p max_scan_retries:=2
```

Monitor:
```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /worble_bot/mission_progress --qos-durability transient_local
```

**Steps:**

1. Set human_detected = false, press START → state = `SCANNING`
2. Wait 4 seconds (timeout: 3s, so recovery triggered at ~3s)
3. State should transition `SCANNING` → `RECOVERING`
4. **During recovery**, publish detections:
   ```bash
   sleep 1  # Let recovery tick
   ros2 topic pub --once /perception/detections std_msgs/msg/String \
     "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
   ```
5. State should transition `RECOVERING` → `SCANNING` (retry) → `READY_TO_MOVE` (success)
6. **No operator button press required**

**Expected Sequence (with timestamps):**

```
T=0s:   SCANNING (human_detected=false, press START)
T=3s:   RECOVERING (timeout detected, retry counter=1)
T=4s:   SCANNING (recovery tick, retry in progress)
T=4.1s: READY_TO_MOVE (detections received, hasEnoughDetections() = true)
```

**Why This Demonstrates HD-Level:**

- **Autonomous recovery:** The recovery branch handles retry logic without operator input
- **Transient failure resilience:** Temporary perception issues are recovered automatically
- **Sophisticated branching:** The recovery branch is a distinct tree node with its own conditions
- **No operator fatigue:** System recovers silently for transient faults

**SEMI-PASS:** If perception is unavailable in testing, the RECOVERING state may be demonstrated without full recovery success. Document the limitation: "Recovery branch executes autonomously; if motion/perception available, full recovery sequence can be demonstrated."

---

## **TC-HD3: Dynamic Subtree Switching Based on Scene Context**

**Criterion:** The behavior tree switches execution context (active branch) based on changing conditions; different subsystems coordinate without explicit state graphs.

**Setup:**

```bash
ros2 launch interaction_execution gui.launch.py \
  launch_motion:=true launch_perception:=false launch_gamification:=false \
  -- -p auto_dispatch_motion:=true
```

Monitor:
```bash
ros2 topic echo /worble_bot/mission_state --qos-durability transient_local
ros2 topic echo /worble_bot/mission_progress --qos-durability transient_local
```

**Steps:**

1. **Scan Branch Active:** Press START → state = `SCANNING`
   - Active branches: Safety guard, scan branch
   - Inactive: motion, recovery, command-driven motion
2. Publish detections → state = `READY_TO_MOVE`
   - Transition to motion branch
   - Active branches: Safety guard, motion branch
   - Inactive: scan, recovery
3. **Motion Branch Active:** Motion dispatched, state = `MOVING`
   - Coordinator now waits for `/wordle_bot/motion_complete`
4. **Inject Failure:** Don't publish motion_complete; let timeout trigger (motion_timeout_s default 20s)
5. After timeout, state should be `MOTION_FAILED`
   - Branches switch: motion branch → failure detection branch
   - Safety branch still active (continuous)
6. **Recovery from Operator:** Press HOME button
   - Command branch activated
   - Coordinator dispatches home goal
   - State = `HOMING`

**Expected Context Switches:**

```
SafetyGuard + ScanBranch (SCANNING)
    ↓ (hasEnoughDetections)
SafetyGuard + MotionBranch (READY_TO_MOVE → MOVING)
    ↓ (motion_complete timeout)
SafetyGuard + FailureDetectionBranch (MOTION_FAILED)
    ↓ (HOME command)
SafetyGuard + MotionBranch (HOMING)
```

**Why This Demonstrates HD-Level:**

- **Context-driven branching:** The coordinator doesn't pre-compute all paths; active branches change dynamically
- **Subsystem coordination:** Scan, motion, and recovery branches run independently but coordinated
- **No hardcoded state graphs:** The tree structure emerges from tick priorities and conditions
- **Seamless transitions:** Switching from scan to motion to failure and back requires no manual reset

**SEMI-PASS or FULL PASS:** If motion subsystem available and auto-dispatch enabled, this shows full dynamic behavior tree execution. (If motion unavailable, demonstrate scan ↔ failure context switching with timeout scenarios instead.)

---

# Evidence Summary & Rubric Mapping

| **Grade** | **Criteria** | **Test Cases** | **Evidence** |
| --- | --- | --- | --- |
| **P** | GUI launches; basic START/STOP commands work | TC-P1 | Screenshots + topic echo of state changes |
| **C** | State machine is consistent; safety prevents unsafe states | TC-C1, TC-C2 | State sequence logs; no invalid transitions |
| **D** | Behavior tree with condition-based transitions; failure detection & recovery; parallel safety monitoring | TC-D1, TC-D2, TC-D3, TC-D4 | State sequence logs showing condition evaluation; mission_progress JSON with retry/failure info; stop_mission signal on safety interrupt |
| **HD** | Sophisticated autonomous recovery; parallel monitoring while active; dynamic context switching; no manual intervention except unrecoverable failures | TC-HD1, TC-HD2, TC-HD3 | mission_progress showing simultaneous updates; recovery without operator button; state transitions reflecting active branch changes |

---

# Quick Reference: Topic Publishing Cheatsheet

**Set human_detected:**
```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
```

**Publish detections (one block):**
```bash
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}'}"
```

**Publish detections (multiple blocks):**
```bash
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02},{\"letter\":\"B\",\"x\":0.40,\"y\":0.10,\"z\":0.02},{\"letter\":\"C\",\"x\":0.50,\"y\":0.10,\"z\":0.02}]}'}"
```

**Simulate mission completion:**
```bash
ros2 topic pub --once /worole_bot/motion_complete std_msgs/msg/Bool "{data: true}"
```

**Monitor latched topics (persist):**
```bash
ros2 topic echo /worble_bot/mission_state --qos-durability transient_local
```

**Monitor non-latched topics (live):**
```bash
ros2 topic echo /worble_bot/stop_mission
ros2 topic echo /perception/human_detected
```

