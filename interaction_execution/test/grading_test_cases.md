# Interaction Execution Grading Test Cases

These tests are written to demonstrate the interaction/execution grading criteria from
Pass through Distinction. They are intentionally topic-level tests so they can be run
with the GUI, the coordinator alone, or a mocked subsystem setup.

## Common Setup

Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select interaction_execution wordlebot_control
source install/setup.bash
```

Run the coordinator without the full GUI:

```bash
ros2 run interaction_execution mission_coordinator_node
```

Useful monitors:

```bash
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo /wordle_bot/mission_progress --qos-durability transient_local
ros2 topic echo /mission/state --qos-durability transient_local
ros2 topic echo /wordle_bot/start_mission
ros2 topic echo /wordle_bot/stop_mission
ros2 topic echo /wordle_bot/set_mission
```

Start monitors for non-latched command topics, such as `/wordle_bot/start_mission`,
`/wordle_bot/stop_mission`, and `/wordle_bot/set_mission`, before running the action
that should publish them.

For tests that require auto dispatch:

```bash
ros2 run interaction_execution mission_coordinator_node --ros-args -p auto_dispatch_motion:=true
```

Example one-block perception payload:

```bash
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}' }"
```

## TC-P1: Basic GUI Command Path

**Criterion:** Basic GUI functionality.

**Setup:**

```bash
ros2 launch interaction_execution gui.launch.py launch_motion:=false launch_perception:=false launch_gamification:=false
```

**Steps:**

1. Press `START` in the GUI.
2. Observe `/wordle_bot/mission_cmd`.
3. Press `STOP`.
4. Observe `/wordle_bot/mission_cmd`.

**Expected result:**

- `START` publishes `std_msgs/String(data="START")`.
- `STOP` publishes `std_msgs/String(data="STOP")`.
- The safety panel changes from idle to active/scanning, then stopped.

**Evidence to record:**

- Screenshot of the GUI safety panel.
- Terminal output from `ros2 topic echo /wordle_bot/mission_cmd`.

## TC-C1: Coordinator START and STOP Without Unexpected State Jumps

**Criterion:** GUI/control functionality with no unintended behaviours.

**Steps:**

```bash
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: START}"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo --once /mission/state --qos-durability transient_local

ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: STOP}"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo --once /mission/state --qos-durability transient_local
```

**Expected result:**

- After `START`, coordinator state becomes `SCANNING`.
- Perception command state becomes `SCANNING`.
- After `STOP`, coordinator state becomes `STOPPED`.
- Perception command state becomes `IDLE`.
- No motion goal is published unless auto dispatch is enabled and detections exist.

## TC-C2: Safety Blocks START When a Human Is Detected

**Criterion:** No unintended behaviours under unsafe input.

**Steps:**

```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: START}"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
```

**Expected result:**

- Coordinator remains or transitions to `STOPPED`.
- The queued `START` command is not executed while human detection is true.
- The mission progress JSON reports the safety step as blocked.

**Evidence to record:**

```bash
ros2 topic echo --once /wordle_bot/mission_progress --qos-durability transient_local
```

## TC-D1: Behaviour Tree Scan Branch Reaches READY_TO_MOVE

**Criterion:** Behaviour tree implemented with condition-based transitions.

**Steps:**

```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: START}"
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}' }"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
```

**Expected result:**

- The coordinator transitions from `SCANNING` to `READY_TO_MOVE`.
- `/wordle_bot/mission_progress` shows scan as active/done and prepare motion as active.

**Why this demonstrates D-level behaviour:**

- The coordinator is not only responding to button presses.
- It evaluates a condition, `hasEnoughDetections()`, before changing branch.

## TC-D2: Auto Dispatch Executes Motion Branch and Handles Completion

**Criterion:** Behaviour tree includes task execution and completion handling.

**Setup:** Run the coordinator with `auto_dispatch_motion:=true`.

**Steps:**

Start these monitors in separate terminals before publishing detections:

```bash
ros2 topic echo /wordle_bot/set_mission
ros2 topic echo /wordle_bot/start_mission
ros2 topic echo /wordle_bot/mission_state --qos-durability transient_local
```

Then publish the inputs:

```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: START}"
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}' }"

ros2 topic pub --once /wordle_bot/motion_complete std_msgs/msg/Bool "{data: true}"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
```

**Expected result:**

- A `PoseArray` is published on `/wordle_bot/set_mission`.
- A `Bool(data=true)` is published on `/wordle_bot/start_mission`.
- Mission state becomes `MOVING`.
- After `/wordle_bot/motion_complete`, mission state returns to `IDLE`.

## TC-D3: Safety Interrupt Has Highest Priority During Motion

**Criterion:** Parallel safety monitoring with interruption behaviour.

**Setup:** Run the coordinator with `auto_dispatch_motion:=true`.

**Steps:**

1. Use TC-D2 steps to reach `MOVING`.
2. Publish a human detection:

```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: true}"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
```

Run this monitor before publishing the human detection if you want command-level
evidence of the interrupt:

```bash
ros2 topic echo /wordle_bot/stop_mission
```

**Expected result for a solid D:**

- Mission state becomes `STOPPED`.
- The coordinator publishes `Bool(data=true)` on `/wordle_bot/stop_mission`.
- Pending motion state is cleared so the robot cannot continue automatically while unsafe.

**Current implementation note:**

- This should pass with only `mission_coordinator_node` running. The GUI should not be required
  to publish the safety stop.

## TC-D4: Condition-Based Recovery After a Safety Stop

**Criterion:** Failure/safety state detection with condition-based recovery.

**Setup:** Run the coordinator with `auto_dispatch_motion:=true`.

**Steps:**

1. Use TC-D3 to trigger a safety stop.
2. Clear the safety condition:

```bash
ros2 topic pub --once /perception/human_detected std_msgs/msg/Bool "{data: false}"
```

3. Request a recovery action:

Start these monitors before publishing `HOME`:

```bash
ros2 topic echo /wordle_bot/set_mission
ros2 topic echo /wordle_bot/start_mission
```

```bash
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: HOME}"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
```

**Expected result:**

- The coordinator accepts recovery only after human detection is false.
- A home mission is published.
- Mission state becomes `HOMING`.

**Optional stronger recovery check:**

```bash
ros2 topic pub --once /wordle_bot/motion_complete std_msgs/msg/Bool "{data: true}"
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
```

Expected final state is `IDLE`.

## TC-D5: Perception Failure Detection and Recovery

**Criterion:** Failure state detection with condition-based recovery behaviour.

Default timeout parameters are `perception_timeout_s:=10.0` and `max_scan_retries:=1`.

**Steps:**

1. Start the mission.
2. Do not publish `/perception/detections`.
3. Wait longer than the configured perception timeout.

```bash
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: START}"
sleep 12
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo --once /wordle_bot/mission_progress --qos-durability transient_local
```

**Expected result:**

- Coordinator enters `RECOVERING` for the first timeout, then retries scanning.
- If the retry also times out, coordinator enters `PERCEPTION_FAILED`.
- Mission progress explains the failure reason.
- Recovery is explicit, for example retry scan once, then wait for operator input.

## TC-D6: Motion Failure Detection and Recovery

**Criterion:** Failure state detection with condition-based recovery behaviour.

Default timeout parameter is `motion_timeout_s:=20.0`.

**Setup:** Run the coordinator with `auto_dispatch_motion:=true`.

**Steps:**

1. Start mission and publish one valid detection.
2. Do not publish `/wordle_bot/motion_complete`.
3. Wait longer than the configured motion timeout.

```bash
ros2 topic pub --once /wordle_bot/mission_cmd std_msgs/msg/String "{data: START}"
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"blocks\":[{\"letter\":\"A\",\"x\":0.30,\"y\":0.10,\"z\":0.02}]}' }"
sleep 20
ros2 topic echo --once /wordle_bot/mission_state --qos-durability transient_local
ros2 topic echo --once /wordle_bot/mission_progress --qos-durability transient_local
```

**Expected result:**

- Coordinator enters `MOTION_FAILED`.
- `/wordle_bot/stop_mission` is published.
- Mission progress identifies that motion completion timed out.

## TC-HD1: Continuous Safety Monitoring While Other Branches Are Active

**Criterion:** Parallel monitoring with interruption behaviours.

**Steps:**

1. Run TC-D2 until `MOVING`.
2. While moving, publish changing perception detections.
3. Publish `/perception/human_detected true`.

**Expected result:**

- Detections can update while the motion branch is active.
- Safety still interrupts immediately.
- Mission progress continues to include the safety step while moving.

## TC-HD2: Priority Selectors for Updated Subsystem Inputs

**Criterion:** Task priority selectors based on safety and subsystem inputs.

**Steps:**

1. Publish `START`.
2. Publish detections to reach `READY_TO_MOVE`.
3. Publish human detection before auto dispatch or before motion completes.

**Expected result:**

- Safety takes priority over scan/motion readiness.
- Coordinator moves to `STOPPED`, not `MOVING`.

## Evidence Matrix

| Criterion | Test cases |
| --- | --- |
| Basic GUI functionality | TC-P1 |
| GUI/control functionality with no unintended behaviours | TC-C1, TC-C2 |
| Behaviour tree implemented | TC-D1, TC-D2 |
| Failure state detection | TC-D5, TC-D6 |
| Condition-based recovery | TC-D4, TC-D5, TC-D6 |
| Parallel safety monitoring/interruption | TC-D3, TC-HD1 |
| Task priority selectors | TC-C2, TC-D3, TC-HD2 |
| Dynamic subtree/context switching | TC-D1, TC-D2, TC-D4 |

## Suggested Demo Order

For a concise assessment demo, run:

1. TC-P1 to show the GUI.
2. TC-D1 to show perception-driven BT transition.
3. TC-D2 to show motion dispatch and completion.
4. TC-D3 to show safety interruption.
5. TC-D4 to show recovery to home.
6. TC-D5 or TC-D6 to show timeout-based failure detection.
