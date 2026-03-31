# Tests for Motion Planning and Control

This document defines the test cases for the WordleBot control subsystem (UR3e robotic arm).
Tests are organized into three concept-based test cases, each containing multiple sub-tests.
All tests are conducted using ROS bags for evidence capture and replay.

Each test maps to one or more grading criteria: **P** (Pass), **C** (Credit), **D** (Distinction),
**HD** (High Distinction), **Perfect**.

> **Note on pending features:** Tests referencing gripper control or mission control
> (Start/Stop/Resume/Abort) describe intended behaviour. Topic names are marked **TBD** and
> will be finalised once those features are implemented.

---

## Prerequisites

The following must be running before any test:

```bash
# 1. UR robot driver
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<IP> ...

# 2. MoveIt stack
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e ...

# 3. WordleBot control node
ros2 launch wordleBot_control wordle_bot.launch.py
```

**Standard bag recording command** (add gripper/mission control topics once implemented):

```bash
ros2 bag record /tf /joint_states \
  /wordle_bot/goal_pose /wordle_bot/motion_complete \
  /planning_scene /move_group/monitored_planning_scene \
  -o <test_name>_bag
```

**Tolerances (from `goal_motion_test.py`):**
- Position: 5 mm
- Orientation: 5°
- Motion timeout: 60 s per waypoint

---

## Terminal 

```bash
colcon test --packages-select wordleBot_control --pytest-args -k tc1_1
```


## Test Case 1 — Key Control Concepts

**Validates:** P, C, D criteria

---

### TC1.1 — Basic Point-to-Point Movement

**Requirement:** Basic trajectory generation and end-effector (EE) tracking to a single pose.

**Assumptions:**
- MoveIt and the UR driver are running and connected to the physical or simulated robot.
- The robot starts from a known safe home position.
- No obstacles are present in the planning scene beyond the default floor and sensor guard.

**Procedure:**
1. Start bag recording.
2. Publish a single goal pose to `/wordle_bot/goal_pose`:
   - Position: `(0.3, 0.25, 0.25)` in `ur_base_link` frame
   - Orientation: `roll=π, pitch=0, yaw=0` (end-effector facing downward)
3. Wait for `/wordle_bot/motion_complete` to publish `true`, or timeout at 60 s.
4. Stop bag recording.
5. Replay bag and assert EE pose using TF lookup (`tool0 → ur_base_link`).

**Pass/Fail Criteria:**
- PASS: EE position is within **5 mm** of the goal; EE orientation is within **5°** of the goal; `motion_complete = true` received within **60 s**.
- FAIL: Any tolerance exceeded, motion timeout reached, or planning/execution error logged.

---

### TC1.2 — Optimised Path Planning

**Requirement:** The controller selects the most efficient trajectory using its cost function (path length, base/wrist rotation weighting).

**Assumptions:**
- The goal pose has multiple valid IK solutions (the cost function has meaningful candidates to choose between).
- The robot starts from the same fixed home position for each run.

**Procedure:**
1. Start bag recording.
2. Publish a goal pose that has multiple valid IK solutions, e.g.:
   - Position: `(-0.2, 0.3, 0.15)` in `ur_base_link` frame
   - Orientation: `roll=π, pitch=0, yaw=0`
3. Allow motion to complete.
4. Stop bag recording.
5. From the bag, extract the full joint trajectory (`/joint_states`).
6. Compute total joint displacement: `Σ |Δq|` across all 6 joints across the trajectory.
7. Compute the minimum-displacement IK solution for the same goal from the same start state (reference baseline).

**Pass/Fail Criteria:**
- PASS: Executed path joint displacement is **≤ 1.5×** the minimum-displacement IK baseline. This confirms the cost function is selecting an efficient plan rather than the first valid solution.
- FAIL: Executed displacement exceeds 1.5× the baseline, indicating no meaningful optimisation.

---

### TC1.3 — Gripper Open/Close

**Requirement:** Gripper control is functional — the gripper opens and closes on command.

**Assumptions:**
- Gripper hardware or simulation is connected and responding.
- Gripper command and state topics are available (**TBD** pending implementation).

**Procedure:**
1. Start bag recording (add gripper topics to record command).
2. Send a gripper **open** command via the gripper command topic (TBD).
3. Wait up to 2 s for gripper state feedback to confirm open position.
4. Send a gripper **close** command.
5. Wait up to 2 s for gripper state feedback to confirm closed position.
6. Stop bag recording.

**Pass/Fail Criteria:**
- PASS: Gripper state feedback confirms **open** within 2 s of open command; confirms **closed** within 2 s of close command.
- FAIL: State feedback does not change, timeout exceeded, or no feedback received.

> **Note:** Topic names TBD. Update this test once gripper interface is implemented.

---

### TC1.4 — Mission Control: Stop / Resume / Abort

**Requirement:** Functioning Start/Stop/Resume/Abort framework for mission execution.

**Assumptions:**
- Mission control topics are available (**TBD** pending implementation).
- The robot can halt motion mid-trajectory without entering a fault state.
- A safe home position is defined for Abort to return to.

**Procedure:**

*Stop / Resume sub-test:*
1. Start bag recording.
2. Send a **Start** command and begin a 3-waypoint sequence.
3. After the robot completes waypoint 1 and begins moving toward waypoint 2, send a **Stop** command.
4. Observe that the arm halts.
5. Send a **Resume** command.
6. Verify the arm continues from its halted pose (not restarting from the beginning).
7. Allow sequence to complete.

*Abort sub-test:*
1. Begin a new 3-waypoint sequence.
2. Send an **Abort** command mid-motion (between waypoints 1 and 2).
3. Verify the arm returns to the safe home position.
4. Stop bag recording.

**Pass/Fail Criteria:**
- PASS (Stop/Resume): Arm halts within **2 s** of Stop command; resumes from the correct halted pose on Resume; completes the sequence successfully.
- PASS (Abort): Arm reaches safe home position within **30 s** of Abort command.
- FAIL: Arm does not halt, resumes from wrong pose, or fails to return home after Abort.

> **Note:** Mission control command/state topic names TBD. Update once interface is implemented.

---

### TC1.5 — Collision Detection and Replanning

**Requirement:** Waypoint movement with collision detection — the planner generates a path that avoids obstacles in the scene.

**Assumptions:**
- The planning scene interface is active and accepting collision objects.
- The obstacle is placed before execution begins (static pre-placed obstacle).

**Procedure:**
1. Add a collision box to the planning scene directly in the straight-line path between waypoint 1 and waypoint 2:
   - Box dimensions: `0.15 m × 0.15 m × 0.3 m`
   - Position: midpoint between the two waypoints
2. Start bag recording (include `/planning_scene`).
3. Publish a 3-waypoint sequence.
4. Allow sequence to complete.
5. Stop bag recording.
6. From the bag, extract the planning scene state and the executed joint trajectory. Verify no trajectory point would place the robot inside the collision box.

**Pass/Fail Criteria:**
- PASS: All 3 waypoints reached within tolerance; no point in the executed trajectory intersects the collision object (verifiable from bag + planning scene).
- FAIL: Planning failure logged, waypoint not reached, or trajectory passes through the collision object.

---

### TC1.6 — Integration: Basic Pick and Place

**Requirement:** All P/C/D criteria working together — waypoint movement, collision avoidance, gripper control, and mission control logic in a single integrated sequence.

**Assumptions:**
- All TC1.1–TC1.5 sub-tests pass individually before this integration test.
- Gripper and mission control interfaces are implemented.
- A physical or simulated object is at the pick location.

**Procedure:**
1. Set up the collision scene (floor + sensor guard + one obstacle between pick and place locations).
2. Start bag recording (all TC1 topics).
3. Send Start command to begin sequence:
   a. Move to pre-pick pose above the object.
   b. Descend to pick pose.
   c. Command gripper **close**.
   d. Lift to clearance height.
   e. Travel to above place location (path must avoid obstacle).
   f. Issue **Stop** command mid-travel — verify arm halts.
   g. Issue **Resume** — verify arm continues.
   h. Descend to place pose.
   i. Command gripper **open**.
4. Stop bag recording.

**Pass/Fail Criteria:**
- PASS: All waypoints reached within tolerance; gripper feedback confirms pick (closed) and place (open); path avoids collision object; Stop/Resume behaves correctly during travel; `motion_complete = true` received at end.
- FAIL: Any individual step fails, gripper does not actuate, path intersects obstacle, or Stop/Resume does not function.

---

## Test Case 2 — Advanced Motion Control

**Validates:** HD criteria

---

### TC2.1 — Design Matrix Goal Ordering Optimisation

**Requirement:** The system uses a design matrix to optimise movement — selecting the most efficient visitation order given a set of goal positions.

**Assumptions:**
- The controller implements goal-ordering optimisation (not just executing goals in the order received).
- A clear "inefficient" input order can be constructed where reordering produces measurably shorter total travel.

**Procedure:**
1. Define 4 goal poses arranged so the optimal visitation order differs from the input order:
   - Input order: far-left → far-right → near-left → near-right (zigzag pattern)
   - Optimal order: far-left → near-left → near-right → far-right (sweep pattern)
2. Start bag recording.
3. Publish all 4 goals and allow the controller to execute.
4. Stop bag recording.
5. From `/joint_states`, compute total joint displacement for the executed order.
6. Compare to the total displacement if goals had been executed in the original input order.

**Pass/Fail Criteria:**
- PASS: Executed visitation order differs from input order; total joint displacement of the executed order is measurably shorter than the naive input order (at least **10% reduction**).
- FAIL: Controller executes goals in input order with no reordering, or reordering does not reduce total displacement.

---

### TC2.2 — Variable Velocity Profiling

**Requirement:** Variable velocity based on safety — arm moves slower near obstacles and faster in open free space.

**Assumptions:**
- The controller applies velocity scaling based on proximity to collision objects.
- The planned trajectory passes through both a constrained zone (near obstacles) and open free space.

**Procedure:**
1. Set up a collision scene with a box obstacle on one side of the workspace.
2. Plan and execute a trajectory from a start pose, past the obstacle, to a goal pose on the far side.
3. Start bag recording.
4. Execute the trajectory.
5. Stop bag recording.
6. Extract velocity profile from `/joint_states`: compute mean joint velocity (rad/s) in two regions:
   - **Near zone:** all trajectory points where EE (from `/tf`) is within 10 cm of the obstacle
   - **Free zone:** all trajectory points where EE is more than 25 cm from any obstacle

**Pass/Fail Criteria:**
- PASS: Mean joint velocity in the **near zone** is at least **20% lower** than mean velocity in the **free zone**.
- FAIL: No measurable velocity difference between zones, or arm moves at uniform speed throughout.

---

### TC2.3 — Robust 4-DOF Pick and Place at Arbitrary Poses

**Requirement:** Robust pick and place control given any 4 DOF pose (X, Y, Z, yaw).

**Assumptions:**
- IK solver can handle the full reachable workspace with arbitrary yaw.
- Gripper interface is implemented.

**Procedure:**
1. Define 3 pick/place pose pairs programmatically with varied X, Y, Z, and yaw:
   - Pair 1: pick at `(0.25, 0.20, 0.05)` yaw=0°, place at `(−0.20, 0.25, 0.10)` yaw=45°
   - Pair 2: pick at `(0.15, −0.20, 0.08)` yaw=90°, place at `(0.30, 0.15, 0.12)` yaw=−30°
   - Pair 3: pick at `(−0.15, 0.28, 0.06)` yaw=180°, place at `(0.20, −0.18, 0.10)` yaw=60°
2. Start bag recording.
3. Execute full pick-and-place (approach → grip → lift → transport → descend → release) at each pose pair.
4. Stop bag recording.

**Pass/Fail Criteria:**
- PASS: All 3 pick poses and all 3 place poses reached within **5 mm / 5°**; gripper closes and opens at correct positions for all 3 pairs; no IK or planning failures logged.
- FAIL: Any pose not reached within tolerance, gripper failure, or IK/planning error at any of the 6 poses.

---

## Test Case 3 — Advanced Collision Avoidance

**Validates:** Perfect criteria

---

### TC3.1 — Active Obstacle Avoidance: Static Obstacle Injected Mid-Execution

**Requirement:** If a hazard enters the environment while the arm is executing a trajectory, the controller detects it and re-generates a safe path.

**Assumptions:**
- The controller monitors the planning scene continuously during execution.
- A replanning mechanism is triggered when the current trajectory becomes invalid.

**Procedure:**
1. Start bag recording (include `/planning_scene` and `/move_group/monitored_planning_scene`).
2. Begin execution of a trajectory to a distant goal pose (expected duration > 5 s).
3. After approximately 2 s (arm in motion), inject a collision box directly in the remaining planned path:
   - Box: `0.2 m × 0.2 m × 0.4 m`
   - Position: along the arm's current planned path, 70% of the way to the goal
4. Continue recording until the arm reaches the goal or times out at 30 s.
5. Stop bag recording.
6. In the bag, identify the timestamp of the obstacle injection. Verify a new trajectory execution starts after that timestamp.

**Pass/Fail Criteria:**
- PASS: A second `/execute_trajectory` event is detectable in the bag after obstacle injection; arm reaches the goal without colliding with the injected obstacle; replanning completes within **15 s** of injection.
- FAIL: Arm continues on the original (now invalid) path, collides with the obstacle, stalls indefinitely, or replanning exceeds the 15 s window.

---

### TC3.2 — Continuous Moving Obstacle Avoidance

**Requirement:** Active collision avoidance against a hazard that is itself moving — the controller continuously maintains a safe path.

**Assumptions:**
- The planning scene can be updated at high frequency (≥ 5 Hz) to simulate a moving obstacle.
- The controller's replanning loop is fast enough to keep up with the obstacle motion.

**Procedure:**
1. Write a test script that continuously publishes collision object position updates at **5 Hz**, moving the obstacle in a sweeping arc that intersects the arm's planned path at all times.
2. Start bag recording.
3. Begin a 2-waypoint trajectory.
4. Simultaneously start the moving obstacle script.
5. Allow the arm to reach the goal or timeout at 60 s.
6. Stop bag recording.
7. For each 200 ms window in the bag, check: does any executed trajectory segment place the EE within the obstacle's bounding box at that timestamp?

**Pass/Fail Criteria:**
- PASS: Arm reaches the final goal within 60 s; no executed trajectory segment intersects the obstacle's position at the corresponding timestamp; multiple replanning events visible in the bag.
- FAIL: Arm collides with the moving obstacle, fails to reach the goal, or produces no replanning events (indicating it is not responding to the obstacle).

---

### TC3.3 — Complete 6-DOF Pick and Place at Arbitrary Poses

**Requirement:** Complete robust pick and place control given any 6 DOF pose (X, Y, Z, roll, pitch, yaw), including non-trivial approach angles.

**Assumptions:**
- IK solver handles the full 6-DOF space including non-vertical approach angles.
- Gripper interface is implemented.
- Active collision scene is present throughout.

**Procedure:**
1. Define 3 pick/place pose pairs with non-trivial roll and pitch (tilted approach angles):
   - Pair 1: pick at `(0.25, 0.20, 0.08)` roll=20°, pitch=−15°, yaw=0°; place at `(−0.20, 0.25, 0.10)` roll=0°, pitch=0°, yaw=45°
   - Pair 2: pick at `(0.15, −0.18, 0.06)` roll=−30°, pitch=10°, yaw=90°; place at `(0.28, 0.15, 0.12)` roll=15°, pitch=−20°, yaw=−30°
   - Pair 3: pick at `(−0.18, 0.25, 0.05)` roll=25°, pitch=−25°, yaw=180°; place at `(0.22, −0.15, 0.10)` roll=−10°, pitch=15°, yaw=60°
2. Ensure the collision scene includes at least one obstacle between each pick and place pair.
3. Start bag recording.
4. Execute full pick-and-place at each pose pair.
5. Stop bag recording.

**Pass/Fail Criteria:**
- PASS: All 3 pick poses and all 3 place poses reached within **5 mm / 5°**; gripper succeeds at each pose; IK solves without failure for all 6-DOF poses; paths avoid collision objects.
- FAIL: Any pose not reached within tolerance, IK failure, gripper failure, or collision with obstacle at any stage.
