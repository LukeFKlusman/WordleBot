# Tests for Motion Planning and Control

This document defines the test cases for the WordleBot control subsystem (UR3e robotic arm).
Tests are organised into three concept-based test cases, each containing multiple sub-tests.
All tests use ROS bags for evidence capture.

Each test maps to one or more grading criteria: **P** (Pass), **C** (Credit), **D** (Distinction),
**HD** (High Distinction), **Perfect**.

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

**Tolerances (shared across all TC1 tests):**
- Position: 5 mm
- Orientation: 5°
- Motion timeout: 60 s per waypoint
- Stop response: 2 s
- Abort response: 30 s
- Gripper response: 2 s

---

## Running Tests

```bash
# Run all TC1 tests
colcon test --packages-select wordleBot_control --pytest-args -k tc1
colcon test-result --verbose

# Run specific sub-tests
python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_1 -s -v
python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_2 -s -v
```

---

## Test Case 1 — Key Control Concepts

**Validates:** P, C, D criteria

---

### TC1.1 — Basic Point-to-Point Movement

**Requirement:** The controller generates a valid trajectory and moves the end-effector (EE) to a single commanded pose within position and orientation tolerance.

**Assumptions:**
- MoveIt and the UR driver are running and connected to the physical or simulated robot.
- The robot starts from a known safe home position.
- No obstacles are present in the planning scene beyond the default floor and sensor guard.

**Procedure:**
1. Start bag recording on topics: `/joint_states`, `/tf`, `/wordle_bot/goal_reached`, `/wordle_bot/mission_complete`.
2. Publish a single-pose `PoseArray` to `/wordle_bot/set_mission`:
   - Position: `(0.3, 0.25, 0.25)` in `world` frame
   - Orientation: `roll=π, pitch=0, yaw=0` (EE facing downward)
3. Publish `Bool(data=True)` to `/wordle_bot/start_mission` to arm the mission.
4. Wait for `/wordle_bot/mission_complete = true`, or timeout at 60 s.
5. Stop bag recording.
6. Query the live TF buffer for the `tool0 → world` transform to get the final EE pose.
7. Read the final joint state from the bag and print joint angles.

**Pass/Fail Criteria:**
- **PASS:** EE position error < **5 mm**; EE orientation error < **5°**; `mission_complete = true` received within **60 s**.
- **FAIL:** Any tolerance exceeded, motion timeout reached, or planning/execution error.

---

### TC1.2 — Optimised Path Planning

**Requirement:** The controller uses a cost function to select efficient trajectories — each leg of a multi-waypoint mission must execute within 10% of the straight-line joint-space displacement baseline.

**Assumptions:**
- The 3 goal poses each have multiple valid IK solutions, giving the cost function meaningful candidates to compare.
- The robot starts from the same fixed home position for each run.

**Procedure:**
1. Start bag recording on topics: `/joint_states`, `/tf`, `/wordle_bot/goal_reached`, `/wordle_bot/mission_complete`.
2. Publish a 3-pose `PoseArray` to `/wordle_bot/set_mission`:
   - Goal 1: `( 0.40,  0.10, 0.25)`, `roll=π`
   - Goal 2: `(-0.30,  0.20, 0.10)`, `roll=π`
   - Goal 3: `( 0.05,  0.20, 0.40)`, `roll=π`
3. Publish `Bool(data=True)` to `/wordle_bot/start_mission`.
4. After each `/wordle_bot/goal_reached` signal, record the EE pose via live TF lookup.
5. Wait for `/wordle_bot/mission_complete`.
6. Stop bag recording.
7. Assert EE pose within tolerance at each of the 3 waypoints.
8. Segment the `/joint_states` trajectory from the bag by `goal_reached` timestamps to isolate each leg.
9. For each leg, compute:
   - **Baseline:** L1 joint-space distance from the leg's start to end joint configuration: `Σ |q_end − q_start|` across all 6 joints.
   - **Actual:** Total L1 joint displacement along the executed trajectory: `Σ Σ |Δq|` across all timesteps and joints.
   - **Ratio:** `actual / baseline` (must be ≤ 1.10).

**Pass/Fail Criteria:**
- **PASS:** All 3 goals reached within **5 mm / 5°**; per-leg displacement ratio ≤ **1.10×** the straight-line baseline for every leg.
- **FAIL:** Any goal tolerance exceeded, timeout, or any leg's displacement ratio exceeds 1.10×.

---

### TC1.3 — Gripper Open/Close

> **Status: Not yet implemented** — gripper command/state topic names are TBD. Test is skipped until the gripper interface is defined.

**Requirement:** Gripper control is functional — the gripper opens and closes on command within 2 s.

**Assumptions:**
- Gripper hardware or simulation is connected and responding.
- Gripper command and state topics are defined (TBD).

**Procedure:**
1. Start bag recording (include gripper command and state topics once defined).
2. Publish a gripper **open** command to the gripper command topic (TBD).
3. Wait up to 2 s for state feedback confirming the open position.
4. Publish a gripper **close** command.
5. Wait up to 2 s for state feedback confirming the closed position.
6. Stop bag recording.

**Pass/Fail Criteria:**
- **PASS:** Gripper state feedback confirms **open** within 2 s of open command; confirms **closed** within 2 s of close command.
- **FAIL:** State feedback does not change, timeout exceeded, or no feedback received.

---

### TC1.4 — Mission Control: Stop / Resume / Abort

> **Status: Not yet implemented** — mission control command/state topic names are TBD. Test is skipped until the mission control interface is defined.

**Requirement:** A functioning Start/Stop/Resume/Abort framework for mission execution.

**Assumptions:**
- Mission control topics are defined (TBD).
- The robot can halt mid-trajectory without entering a fault state.
- A safe home position is defined for Abort to return to.

**Procedure:**

*Stop/Resume sub-test:*
1. Start bag recording.
2. Publish a 3-waypoint mission via `/wordle_bot/set_mission` with waypoints:
   - `( 0.3,  0.25, 0.25)`, `(-0.2, 0.3, 0.15)`, `( 0.2, 0.25, 0.20)` — all `roll=π`
3. Arm the mission via `/wordle_bot/start_mission`.
4. After waypoint 1 completes, send a **Stop** command via the mission control topic (TBD).
5. Record the EE pose at the moment Stop was issued.
6. Wait 2 s and verify the EE has not moved (arm has halted).
7. Send a **Resume** command; verify the arm continues toward waypoint 2 (not restarting from the beginning).
8. Allow the full 3-waypoint sequence to complete.

*Abort sub-test:*
1. Begin a new 3-waypoint sequence.
2. Send an **Abort** command mid-motion (between waypoints 1 and 2).
3. Verify the arm returns to the safe home position within 30 s.
4. Stop bag recording.

**Pass/Fail Criteria:**
- **PASS (Stop/Resume):** Arm halts within **2 s** of Stop command; resumes from the correct halted pose on Resume; completes the full sequence.
- **PASS (Abort):** Arm reaches safe home position within **30 s** of Abort command.
- **FAIL:** Arm does not halt, resumes from wrong pose, fails to return home after Abort, or topics are unavailable.

---

### TC1.5 — Collision Detection and Replanning

> **Status: Not yet implemented** — planning scene injection interface is TBD. Test is skipped until the interface is defined.

**Requirement:** The planner generates a collision-free path when an obstacle is present in the planning scene between two waypoints.

**Assumptions:**
- The MoveIt planning scene interface is active and accepting collision objects.
- The obstacle is injected before execution begins (static pre-placed obstacle).

**Procedure:**
1. Add a collision box to the planning scene in the straight-line path between waypoint 1 and waypoint 2:
   - Box dimensions: `0.15 m × 0.15 m × 0.3 m`, at the midpoint between the two waypoints
2. Start bag recording (include `/planning_scene`).
3. Publish a 3-waypoint sequence via `/wordle_bot/set_mission` with waypoints:
   - `( 0.3,  0.25, 0.25)`, `(-0.2, 0.3, 0.15)`, `( 0.2, 0.25, 0.20)` — all `roll=π`
4. Arm the mission and allow it to complete.
5. Stop bag recording.
6. Extract the planning scene state and the executed `/joint_states` trajectory from the bag.
7. For each trajectory point, verify that no joint configuration places the robot inside the collision box.
8. Remove the obstacle from the planning scene.

**Pass/Fail Criteria:**
- **PASS:** All 3 waypoints reached within **5 mm / 5°**; no point in the executed trajectory intersects the collision box (verified from bag + planning scene).
- **FAIL:** Planning failure logged, any waypoint not reached within tolerance, or any trajectory point intersects the obstacle.

---

### TC1.6 — Integration: Basic Pick and Place

> **Status: Not yet implemented** — depends on gripper (TC1.3) and mission control (TC1.4) interfaces, both TBD. Test is skipped until those interfaces are defined.

**Requirement:** All P/C/D criteria operating together — point-to-point movement, collision avoidance, gripper control, and Stop/Resume mission control in a single integrated sequence.

**Assumptions:**
- TC1.1 through TC1.5 each pass individually.
- Gripper and mission control interfaces are implemented.
- A physical or simulated object is at the pick location.

**Procedure:**
1. Add a collision obstacle between the pick and place locations in the planning scene.
2. Start bag recording (all TC1 topics + gripper + mission control topics).
3. Execute the following sequence:
   a. Move to pre-pick pose: `( 0.3, 0.20, 0.15)`, `roll=π`
   b. Descend to pick pose: `( 0.3, 0.20, 0.05)`, `roll=π`
   c. Command gripper **close** and wait for closed confirmation.
   d. Lift to clearance height: `( 0.3, 0.20, 0.20)`, `roll=π`
   e. Begin travel toward place pose: `(-0.2, 0.25, 0.05)`, `roll=π` (path must avoid obstacle).
   f. Issue **Stop** command mid-travel — verify arm halts within 2 s.
   g. Issue **Resume** — verify arm continues toward the place pose.
   h. Descend to place pose and command gripper **open**.
4. Stop bag recording.

**Pass/Fail Criteria:**
- **PASS:** All 4 poses reached within **5 mm / 5°**; gripper state feedback confirms closed at pick and open at place; planned path avoids the collision obstacle; Stop/Resume behaves correctly during travel; `mission_complete = true` received at end.
- **FAIL:** Any pose not reached within tolerance, gripper does not actuate, path intersects the obstacle, or Stop/Resume does not function.

---

## Test Case 2 — Advanced Motion Control

**Validates:** HD criteria

---

### TC2.1 — Design Matrix Goal Ordering Optimisation

**Requirement:** The system uses a design matrix to select the most efficient visitation order for a set of goal positions, rather than executing them in the order received.

**Assumptions:**
- The controller implements goal-ordering optimisation.
- A clear "inefficient" input order can be constructed where reordering produces measurably shorter total travel.

**Procedure:**
1. Define 4 goal poses in a zigzag input order (far-left → far-right → near-left → near-right) where the optimal sweep order (far-left → near-left → near-right → far-right) is measurably shorter.
2. Start bag recording.
3. Publish all 4 goals and allow the controller to execute.
4. Stop bag recording.
5. Extract the `/joint_states` trajectory from the bag and compute total L1 joint displacement for the executed order.
6. Compute total displacement for the original input order as a baseline for comparison.

**Pass/Fail Criteria:**
- **PASS:** Executed visitation order differs from input order; total joint displacement of the executed order is at least **10% less** than the naive input order.
- **FAIL:** Controller executes goals in input order with no reordering, or reordering does not reduce total displacement.

---

### TC2.2 — Variable Velocity Profiling

**Requirement:** The arm moves slower near obstacles and faster in open free space — velocity is scaled based on proximity to collision objects.

**Assumptions:**
- The controller applies velocity scaling based on proximity to planning scene objects.
- The planned trajectory passes through both a constrained zone (near obstacles) and open free space.

**Procedure:**
1. Add a collision box obstacle to one side of the workspace.
2. Plan and execute a trajectory from a start pose, past the obstacle, to a goal pose on the far side.
3. Start bag recording.
4. Execute the trajectory.
5. Stop bag recording.
6. Extract velocity profile from `/joint_states` and TF data. Compute mean joint velocity (rad/s) in two regions:
   - **Near zone:** trajectory points where EE (from `/tf`) is within **10 cm** of the obstacle.
   - **Free zone:** trajectory points where EE is more than **25 cm** from any obstacle.

**Pass/Fail Criteria:**
- **PASS:** Mean joint velocity in the **near zone** is at least **20% lower** than mean velocity in the **free zone**.
- **FAIL:** No measurable velocity difference between zones, or arm moves at uniform speed throughout.

---

### TC2.3 — Robust 4-DOF Pick and Place at Arbitrary Poses

**Requirement:** Robust pick and place given any 4-DOF pose (X, Y, Z, yaw), including poses with varied yaw angles across the reachable workspace.

**Assumptions:**
- IK solver handles the full reachable workspace with arbitrary yaw.
- Gripper interface is implemented.

**Procedure:**
1. Define 3 pick/place pose pairs with varied X, Y, Z, and yaw:
   - Pair 1: pick `(0.25, 0.20, 0.05)` yaw=0°, place `(−0.20, 0.25, 0.10)` yaw=45°
   - Pair 2: pick `(0.15, −0.20, 0.08)` yaw=90°, place `(0.30, 0.15, 0.12)` yaw=−30°
   - Pair 3: pick `(−0.15, 0.28, 0.06)` yaw=180°, place `(0.20, −0.18, 0.10)` yaw=60°
2. Start bag recording.
3. Execute full pick-and-place (approach → grip → lift → transport → descend → release) for each pair.
4. Stop bag recording.

**Pass/Fail Criteria:**
- **PASS:** All 3 pick poses and all 3 place poses reached within **5 mm / 5°**; gripper closes and opens correctly at each pair; no IK or planning failures logged.
- **FAIL:** Any pose not reached within tolerance, gripper failure, or IK/planning error at any of the 6 poses.

---

## Test Case 3 — Advanced Collision Avoidance

**Validates:** Perfect criteria

---

### TC3.1 — Active Obstacle Avoidance: Static Obstacle Injected Mid-Execution

**Requirement:** If a hazard enters the environment while the arm is executing a trajectory, the controller detects it and re-generates a safe path without stopping.

**Assumptions:**
- The controller monitors the planning scene continuously during execution.
- A replanning mechanism is triggered when the current trajectory becomes invalid.

**Procedure:**
1. Start bag recording (include `/planning_scene` and `/move_group/monitored_planning_scene`).
2. Begin execution of a trajectory to a distant goal (expected duration > 5 s).
3. After approximately 2 s (arm in motion), inject a collision box into the planning scene directly in the remaining planned path:
   - Box: `0.2 m × 0.2 m × 0.4 m`, positioned 70% of the way along the arm's current planned path to the goal.
4. Continue recording until the arm reaches the goal or times out at 30 s.
5. Stop bag recording.
6. In the bag, identify the timestamp of obstacle injection. Verify that a new trajectory execution event starts after that timestamp.

**Pass/Fail Criteria:**
- **PASS:** A second `/execute_trajectory` event is detectable in the bag after obstacle injection; arm reaches the goal without colliding with the injected obstacle; replanning completes within **15 s** of injection.
- **FAIL:** Arm continues on the original (now invalid) path, collides with the obstacle, stalls indefinitely, or replanning exceeds 15 s.

---

### TC3.2 — Continuous Moving Obstacle Avoidance

**Requirement:** The controller continuously maintains a safe path against a hazard that is itself moving, replanning repeatedly to avoid it.

**Assumptions:**
- The planning scene can be updated at ≥ 5 Hz to simulate a moving obstacle.
- The controller's replanning loop is fast enough to keep up with obstacle motion.

**Procedure:**
1. Write a test script that publishes collision object position updates at **5 Hz**, sweeping the obstacle in an arc that intersects the arm's planned path at all times.
2. Start bag recording.
3. Begin a 2-waypoint trajectory and simultaneously start the moving obstacle script.
4. Allow the arm to reach the goal or timeout at 60 s.
5. Stop bag recording.
6. For each 200 ms window in the bag, verify that no executed trajectory segment places the EE within the obstacle's bounding box at the corresponding timestamp.

**Pass/Fail Criteria:**
- **PASS:** Arm reaches the final goal within **60 s**; no executed segment intersects the obstacle at the corresponding timestamp; multiple replanning events are visible in the bag.
- **FAIL:** Arm collides with the moving obstacle, fails to reach the goal, or produces no replanning events.

---

### TC3.3 — Complete 6-DOF Pick and Place at Arbitrary Poses

**Requirement:** Complete robust pick and place given any 6-DOF pose (X, Y, Z, roll, pitch, yaw), including non-trivial approach angles, with an active collision scene throughout.

**Assumptions:**
- IK solver handles the full 6-DOF space including non-vertical approach angles.
- Gripper interface is implemented.
- Active collision scene is present throughout.

**Procedure:**
1. Define 3 pick/place pose pairs with non-trivial roll and pitch:
   - Pair 1: pick `(0.25, 0.20, 0.08)` roll=20°, pitch=−15°, yaw=0°; place `(−0.20, 0.25, 0.10)` roll=0°, pitch=0°, yaw=45°
   - Pair 2: pick `(0.15, −0.18, 0.06)` roll=−30°, pitch=10°, yaw=90°; place `(0.28, 0.15, 0.12)` roll=15°, pitch=−20°, yaw=−30°
   - Pair 3: pick `(−0.18, 0.25, 0.05)` roll=25°, pitch=−25°, yaw=180°; place `(0.22, −0.15, 0.10)` roll=−10°, pitch=15°, yaw=60°
2. Ensure the collision scene includes at least one obstacle between each pick and place pair.
3. Start bag recording.
4. Execute full pick-and-place at each pose pair.
5. Stop bag recording.

**Pass/Fail Criteria:**
- **PASS:** All 3 pick poses and all 3 place poses reached within **5 mm / 5°**; gripper succeeds at each pose; IK solves without failure for all 6-DOF poses; paths avoid all collision objects.
- **FAIL:** Any pose not reached within tolerance, IK failure, gripper failure, or collision with any obstacle.
