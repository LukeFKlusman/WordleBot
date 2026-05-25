# Pick/Place Planning Decisions

## Context

Pick and place was mechanically reachable and repeatable, but MTC planning was producing unpredictable numbers of complete solutions. Some solutions were usable, while others contained unnecessary wrist spinning. The issue was not simply reachability or planning time; it was how MTC combined exact pose generation, IK diversity, OMPL candidate generation, and solution selection.

## Decisions Made

### Exact yaw was tried, but is no longer the active policy

Pick poses must use the yaw of the detected object, and place poses must use the requested place yaw, currently zero yaw from high-level control.

To enforce this:

- Pick still uses `GenerateGraspPose`, but yaw sampling was reduced to one exact object-frame yaw.
- Place was changed from `GeneratePlacePose` to `GeneratePose`, because `GeneratePlacePose` intentionally rotates box objects about Z.
- Pick/place quaternions are normalized before planning.

Result: this made planning much less reliable and slower, even though the poses were mechanically reachable. The likely cause is that exact yaw reduced MTC's candidate space too aggressively:

- `GenerateGraspPose` with `angle_delta = 2π` emits only one grasp orientation.
- `GeneratePose` for place emits only one exact target pose.
- A yaw cost cannot help if the generator never produces alternative candidates.

Decision: the hard exact-yaw approach is superseded by the soft place-yaw approach below.

### Place yaw is a strong preference, not a hard constraint

The active policy is now:

- Pick yaw is not scored, because any stable grasp is acceptable if it can place the letter correctly.
- Place yaw is the correctness target.
- Wrong-yaw place alternatives are allowed as fallback if no better complete solution is found.

Implementation:

- Pick uses `GenerateGraspPose` with configurable `pick_place.grasp_angle_delta`, defaulting back to `π/12`.
- Place uses `GeneratePlacePose` again, so MTC can generate the normal box-symmetry yaw alternatives.
- The requested `place_pose` orientation is passed into `GeneratePlacePose`; it is no longer forced to identity.
- Pick/place quaternions are still normalized before planning.

Decision: restore MTC candidate breadth first, then bias final solution selection toward the requested place yaw.

### Keep MTC for now

MTC was kept because its staged UI and scene handling are useful for pick/place debugging. A full MoveGroupInterface rewrite remains the fallback if MTC continues to be unreliable.

Decision: improve MTC first, rather than immediately replacing it.

### Keep broader IK diversity

Because pick/place planning is sensitive to the number and spread of IK states available to MTC:

- `grasp_min_solution_distance` was lowered.
- `place_min_solution_distance` was lowered.
- IK solution counts remain high.

Decision: keep generous IK diversity, but now combine it with broader grasp/place pose generation instead of relying on exact pose targets only.

### Score trajectory quality explicitly

The custom MTC planner now scores OMPL candidates by:

- total joint motion
- wrist spin penalty
- optional rejection if wrist spin exceeds a threshold

Decision: avoid relying only on OMPL success/failure or default MTC cost, because a successful trajectory can still be a poor robot motion.

### Select the best complete task solution

The controller no longer executes `task.solutions().front()` blindly. It scores every complete pick/place task solution and selects the best combined score.

The score now includes:

- total joint motion
- wrist spin penalty
- soft place-yaw penalty
- a small MTC cost tie-breaker

Place-yaw penalty is zero inside the configured tolerance. Outside tolerance, the penalty is quadratic in the shortest yaw error beyond the tolerance:

```text
penalty = place_yaw_penalty_weight * max(0, yaw_error - place_yaw_tolerance)^2
```

Decision: MTC solution order is not trusted as the final quality ranking.

### Accept good-enough plans early

Planning now runs incrementally. If a complete pick/place solution scores under:

```yaml
pick_place.accept_solution_score_threshold: 25.0
```

the controller accepts it and moves on instead of continuing to search for more complete solutions.

Decision: a sufficiently good plan is better than spending extra time searching for marginal improvement.

### Restore the wrist_1 IK filter

Temporarily removing the custom `wrist_1_joint` IK filter made behavior worse, so it was restored.

Decision: keep the existing wrist_1 filter for now, while still allowing MoveIt robot-model limits to apply normally.

## Important Current Parameters

`config/wordle_bot_controller.yaml`

- `pick_place.grasp_min_solution_distance`
- `pick_place.place_min_solution_distance`
- `pick_place.grasp_angle_delta`
- `pick_place.task_solution_target_count`
- `pick_place.accept_solution_score_threshold`
- `pick_place.solution_wrist_spin_weight`
- `pick_place.solution_wrist_spin_reject_threshold`
- `pick_place.place_yaw_tolerance`
- `pick_place.place_yaw_penalty_weight`

`config/wordle_mtc_planner.yaml`

- `wordle_mtc_planner.candidate_plans`
- `wordle_mtc_planner.trajectory_wrist_spin_weight`
- `wordle_mtc_planner.trajectory_wrist_spin_reject_threshold`
- `wordle_mtc_planner.ik_wrist_1_min`
- `wordle_mtc_planner.ik_wrist_1_max`

## Latest Implementation Notes

The soft place-yaw change was implemented in:

- `src/wordle_bot_controller.cpp`
- `src/wordle_bot_control_node.cpp`
- `config/wordle_bot_controller.yaml`

Build verification:

```bash
colcon build --packages-select wordlebot_control
```

Result: build succeeded. Colcon reported the existing underlay override warning for `wordlebot_control`, but compilation completed successfully.

## Fallback Option

If MTC remains too unpredictable, the fallback is to implement pick/place using explicit MoveGroupInterface phases:

- free-space pre-grasp
- Cartesian approach
- close and attach
- Cartesian lift
- free-space pre-place
- release and retreat

That would provide more direct control over repeated planning and candidate selection, but would lose much of the MTC staged visualization unless recreated separately.
