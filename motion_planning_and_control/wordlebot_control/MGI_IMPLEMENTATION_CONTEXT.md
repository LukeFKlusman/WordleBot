# MGI Implementation Context — For Scan and Sweep Implementation

This document summarises the work done to implement a `MoveGroupInterface`-based motion path
alongside the existing MoveIt Task Constructor (MTC) path. It is intended to give a future agent
enough context to add Cartesian scan-and-sweep functionality using the same infrastructure.

---

## What Was Built

### The Problem

The existing `planMoveToGoal` / `executePlannedMoveToGoal` functions use MoveIt Task Constructor
(MTC) and only succeed ~50% of the time. They provide little control over IK solving or trajectory
selection. A parallel path using `MoveGroupInterface` was added to give explicit control over both.

### Compile-Time Toggle

```cpp
// include/wordlebot_control/wordle_bot_controller.hpp  (~line 199)
static constexpr bool USE_MTC_FOR_GOALS = true;
```

- `true` → existing MTC plan-all-then-execute-all behaviour (unchanged)
- `false` → new `MoveGroupInterface` sequential plan-then-execute per goal

The `missionLoop` in `wordle_bot_control_node.cpp` uses `if constexpr` to branch on this flag,
so the dead branch is compiled out entirely.

---

## New Infrastructure Added

### 1. PlanningSceneMonitor (`psm_`)

A `planning_scene_monitor::PlanningSceneMonitorPtr psm_` was added as a private member of
`WordleBotController`. It is initialised in the constructor:

```cpp
// src/wordle_bot_controller.cpp  (constructor, ~line 39)
psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
psm_->startSceneMonitor("/monitored_planning_scene");
psm_->requestPlanningSceneState("/get_planning_scene");
```

This gives direct access to the live planning scene (including the floor, sensor guard, and any
letter collision objects) for collision checking without going through a service call.
**This is the key piece that enables collision-aware IK and is reusable for scan-and-sweep.**

### 2. `computeBestIK`

```cpp
// src/wordle_bot_controller.cpp  (~line 904)
std::vector<double> WordleBotController::computeBestIK(
    const moveit::core::RobotStatePtr & current_state,
    const geometry_msgs::msg::Pose & target_pose)
```

- 15 IK attempts: first 5 seeded from warm-start config, remaining 10 random
- Warm-start: `[1.1345, -1.5708, 1.5708, -1.5708, -1.5708, 1.1345]` rad
  = `[65°, -90°, 90°, -90°, -90°, 65°]`
- **Collision-aware**: uses `LockedPlanningSceneRO(psm_)` + `isCollisionFree` lambda passed to
  `setFromIK` — only collision-free IK solutions are accepted
- 2π normalisation per revolute joint (prevents 10+ radian spinning trajectories)
- Wrist-3 clamped to `[-π, π]` (required by UR RTDE hardware interface)
- Tip link: `"gripper_tcp"`
- No shoulder rejection (unlike the old code in `oldmovetofunct.md`)
- Cost: `2.0 * movement_cost + 0.3 * functional_penalty`

### 3. `generateCandidatePlans` and `selectBestPlan`

```cpp
// src/wordle_bot_controller.cpp  (~line 1020 and ~line 1043)
std::vector<MoveGroupInterface::Plan> generateCandidatePlans(int num_attempts);
MoveGroupInterface::Plan selectBestPlan(const std::vector<MoveGroupInterface::Plan>& plans);
```

- `generateCandidatePlans`: calls `move_group_.plan()` N times, returns successes
- `selectBestPlan`: returns the plan with minimum `computeTotalJointDisplacement` (static helper
  already existed on the class)

### 4. `moveToGoal`

```cpp
// src/wordle_bot_controller.cpp  (~line 1062)
bool WordleBotController::moveToGoal(const geometry_msgs::msg::Pose & goal_pose)
```

Orchestrator: get current state → `computeBestIK` → `setJointValueTarget` →
`generateCandidatePlans(5)` → `selectBestPlan` → `move_group_.execute`. No path constraints.

---

## Existing Scan-and-Sweep Code (MTC-based, currently not reliable)

```cpp
// src/wordle_bot_controller.cpp  (~line 1200+)
moveit::task_constructor::Task WordleBotController::createScanAndSweepTask(...)
bool WordleBotController::runScanAndSweep(const std::vector<geometry_msgs::msg::Pose>& poses)
```

The current MTC scan-and-sweep uses:
- Free-space OMPL move to pose 0 (via `planMoveToGoal`)
- A single MTC task for poses 1–3 using `Connect(CartesianPath)` + `ComputeIK(GeneratePose)`

This is unreliable for the same reasons as `planMoveToGoal`. It needs to be replaced with a
`MoveGroupInterface` Cartesian path approach using the same infrastructure built above.

### Scan poses config

Four poses loaded from `config/scan_sweep_poses.yaml` via launch file, available in
`wordle_bot_control_node.cpp` as `scan_sweep_poses_[0..3]`. Passed to `runScanAndSweep` as a
`std::vector<geometry_msgs::msg::Pose>`.

### Callback

```cpp
// src/wordle_bot_control_node.cpp  (scanAndSweepCallback)
void WordleBotControlNode::scanAndSweepCallback(const std_msgs::msg::Bool::SharedPtr msg)
```

Calls `controller_->runScanAndSweep(poses)` in a detached thread. Sets `mission_running_` and
publishes `RUNNING` / `IDLE` state around it. This callback does NOT go through `missionLoop`.

---

## How to Implement Scan and Sweep with Cartesian Planning

The approach mirrors the MGI goal path: replace the MTC task with direct `MoveGroupInterface` calls.
The key API is `computeCartesianPath`:

```cpp
// MoveGroupInterface API
double MoveGroupInterface::computeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double eef_step,           // max step size in Cartesian space (e.g. 0.01 m)
    double jump_threshold,     // 0.0 disables jump check; or ~5.0 for safety
    moveit_msgs::msg::RobotTrajectory& trajectory,
    bool avoid_collisions = true);
// Returns fraction of path completed (1.0 = full path planned)
```

### Suggested implementation of new `runScanAndSweep`

```
1. Move to poses[0] using moveToGoal(poses[0])
   — this handles IK solving + joint-space planning, same as regular goal navigation

2. For each segment poses[i] → poses[i+1] (i = 0, 1, 2):
   a. Build waypoints = {poses[i+1]}  (one-segment Cartesian move)
   b. Call computeCartesianPath(waypoints, eef_step=0.01, jump_threshold=0.0, trajectory)
   c. If fraction < 0.95: fall back to moveToGoal(poses[i+1]) — free-space IK plan
   d. Otherwise: move_group_.execute(trajectory)
   e. Dwell for scan_sweep_dwell_time_ seconds

3. Return to home via returnToHome()
```

This gives:
- Cartesian sweeps between scan poses (camera stays on a predictable path)
- Fallback to free-space planning if Cartesian fails (e.g. near singularity)
- No MTC dependency

### Relevant constants

```cpp
// Group and end effector (same as moveToGoal)
const std::string arm_group = "ur_onrobot_manipulator";  // move_group_ group
// End effector link: "gripper_tcp"  (confirmed from robot model link list)

// Cartesian path parameters (tune to robot speed)
constexpr double kCartesianEefStep       = 0.01;   // 1 cm step
constexpr double kCartesianJumpThreshold = 0.0;    // 0 = no jump check
constexpr double kCartesianMinFraction   = 0.95;   // fraction to accept as success
```

### Reuse from existing infrastructure

- `psm_` (PlanningSceneMonitor) — collision-aware Cartesian paths via `avoid_collisions=true`
- `moveToGoal` — free-space fallback and pose-0 approach
- `move_group_.setMaxVelocityScalingFactor` / `setMaxAccelerationScalingFactor` — slow down
  for Cartesian segments if needed
- `computeTotalJointDisplacement` — static helper for plan diagnostics

### What NOT to reuse

- `computeBestIK` — not needed for Cartesian planning (the path is Cartesian, IK is solved
  internally by `computeCartesianPath`)
- `generateCandidatePlans` / `selectBestPlan` — not needed; Cartesian path is deterministic

---

## File Map

| File | Purpose |
|---|---|
| `include/wordlebot_control/wordle_bot_controller.hpp` | Class interface, toggle, all method declarations, `psm_` member |
| `src/wordle_bot_controller.cpp` | All implementations — constructor (PSM init), computeBestIK, moveToGoal, existing runScanAndSweep |
| `src/wordle_bot_control_node.cpp` | ROS2 node — scanAndSweepCallback, missionLoop with if-constexpr toggle |
| `config/scan_sweep_poses.yaml` | Four scan poses as `[x, y, z, roll, pitch, yaw]` |
| `config/ik_warm_start.yaml` | Warm-start joint values loaded by launch file |
| `launch/wordle_bot_mtc.launch.py` | Loads both yaml configs, starts `wordlebot_control_node` |
| `src/oldmovetofunct.md` | Reference for old MGI implementation (DO NOT copy — outdated group names, hard limits) |

---

## Key Notes for Future Agent

1. **Group name is `"ur_onrobot_manipulator"`** — not `"ur_manipulator"` (the old name before the
   gripper was added). Using the wrong group name silently fails.

2. **End effector link is `"gripper_tcp"`** (lowercase) — confirmed from robot model link list in
   terminal logs. The `move_group_.getEndEffectorLink()` returns `"gripper_tcp"`.

3. **`psm_` is already initialised** in the constructor and kept in sync with `/monitored_planning_scene`.
   Cartesian path planning with `avoid_collisions=true` will automatically use the live scene
   (floor, sensor guard) — no extra setup needed.

4. **Do NOT add another PlanningSceneMonitor** — `psm_` already exists on the controller. Reuse it.

5. **`scanAndSweepCallback` runs in its own detached thread** — it does NOT use the `missionLoop`
   condition variable pattern. It sets `mission_running_` directly. Keep this pattern.

6. **`USE_MTC_FOR_GOALS` toggle only affects goal navigation** — scan-and-sweep has its own
   callback and `runScanAndSweep` function. The toggle does not need to be extended to scan-and-sweep
   unless specifically required.
