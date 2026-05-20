std::vector<double> WordleBotController::computeBestIK(const moveit::core::RobotStatePtr & current_state,
                                                       const geometry_msgs::msg::Pose & target_pose)
{
  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);
  if (joint_model_group == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: joint model group '%s' not found in robot model.", kPlanningGroup);
    return {};
  }
  if (current_state == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: current robot state is null.");
    return {};
  }

  // Verify kinematics solver is available before attempting IK
  const kinematics::KinematicsBaseConstPtr & solver = joint_model_group->getSolverInstance();
  if (!solver) {
    RCLCPP_ERROR(LOGGER,
      "computeBestIK: No kinematics solver loaded for group '%s'. "
      "Ensure kinematics.yaml is passed as a node parameter and the plugin is installed.",
      kPlanningGroup);
    return {};
  }
  RCLCPP_DEBUG(LOGGER, "computeBestIK: Using kinematics solver '%s'.", solver->getGroupName().c_str());

  std::vector<double> current_joint_values;
  current_state->copyJointGroupPositions(joint_model_group, current_joint_values);

  // Find shoulder_lift_joint index so we can normalise its angle to satisfy the path constraint.
  // The path constraint applied during planning is centred at -π/2 with ±110° tolerance.
  const double shoulder_center = -M_PI / 2.0;
  const double shoulder_tol    = M_PI / 180.0 * 110.0;
  const auto & joint_names = joint_model_group->getVariableNames();
  const int shoulder_idx = jointNameIndex(joint_names, "shoulder_lift_joint");

  std::vector<double> best_joint_values;
  double best_cost = std::numeric_limits<double>::infinity();
  int ik_successes = 0;
  int constraint_rejects = 0;

  // Functional position [0°, -135°, -90°, -45°, 0°, 0°] and per-joint weights.
  // The quadratic penalty steers IK selection toward a comfortable canonical posture.
  static const double q_func[6] = {0.0, -2.3562, -1.5708, -0.7854, 0.0, 0.0};
  static const double w_func[6] = {0.3,  0.5,     0.5,     0.5,    0.3,  0.3};

  for (int attempt = 0; attempt < 15; ++attempt) {
    moveit::core::RobotState ik_state(*current_state);
    ik_state.setToRandomPositions(joint_model_group);

    if (!ik_state.setFromIK(joint_model_group, target_pose, 0.1)) {
      // RCLCPP_INFO(LOGGER, "computeBestIK: attempt %d failed IK.", attempt);
      continue;
    }

    ++ik_successes;
    std::vector<double> candidate_joint_values;
    ik_state.copyJointGroupPositions(joint_model_group, candidate_joint_values);

    // Normalise every revolute joint to the 2π-equivalent numerically closest to the current
    // joint state. Without this, the planner receives a raw IK angle like 4.24 rad when the
    // robot is at -6.12 rad — they are equivalent mod 2π but 10.36 rad apart numerically,
    // causing the planner to generate a huge spinning trajectory.
    const auto & active_joints = joint_model_group->getActiveJointModels();
    for (std::size_t i = 0; i < candidate_joint_values.size() && i < active_joints.size(); ++i) {
      if (active_joints[i]->getType() != moveit::core::JointModel::REVOLUTE) {
        continue;
      }
      const double curr    = current_joint_values[i];
      const double raw     = candidate_joint_values[i];
      double best_norm     = raw;
      double best_dist     = std::abs(raw - curr);
      for (int k = -3; k <= 3; ++k) {
        const double offset = raw + k * 2.0 * M_PI;
        const double dist   = std::abs(offset - curr);
        if (dist < best_dist && active_joints[i]->satisfiesPositionBounds(&offset)) {
          best_dist = dist;
          best_norm = offset;
        }
      }
      if (std::abs(best_norm - raw) > 1e-6) {
        // RCLCPP_INFO(LOGGER,
        //   "computeBestIK: attempt %d normalised joint '%s': raw=%.4f -> norm=%.4f rad.",
        //   attempt, active_joints[i]->getName().c_str(), raw, best_norm);
      }
      candidate_joint_values[i] = best_norm;
    }

    // Clamp wrist_3_joint to [-π, π].
    // The UR RTDE hardware interface represents this continuous joint in [-π, π]. Allowing
    // values outside that range causes a 6.283 rad (2π) PATH_TOLERANCE_VIOLATED abort on
    // the scaled_joint_trajectory_controller when the controller's tracked state disagrees
    // with the planned trajectory start by exactly one full revolution.
    const int wrist3_idx = jointNameIndex(
      std::vector<std::string>(joint_names.begin(), joint_names.end()), "wrist_3_joint");
    if (wrist3_idx >= 0 && wrist3_idx < static_cast<int>(candidate_joint_values.size())) {
      double & w3 = candidate_joint_values[wrist3_idx];
      const double raw_w3 = w3;
      while (w3 > M_PI)  w3 -= 2.0 * M_PI;
      while (w3 < -M_PI) w3 += 2.0 * M_PI;
      if (std::abs(w3 - raw_w3) > 1e-6) {
        // RCLCPP_INFO(LOGGER,
        //   "computeBestIK: attempt %d clamped wrist_3 to [-pi,pi]: %.4f -> %.4f rad.",
        //   attempt, raw_w3, w3);
      }
    }

    // Reject if shoulder_lift_joint is outside the path constraint range after normalisation.
    if (shoulder_idx >= 0 && shoulder_idx < static_cast<int>(candidate_joint_values.size())) {
      const double sh = candidate_joint_values[shoulder_idx];
      if (std::abs(sh - shoulder_center) > shoulder_tol) {
        // RCLCPP_INFO(LOGGER,
        //   "computeBestIK: attempt %d rejected — shoulder %.4f rad outside constraint [%.4f, %.4f].",
        //   attempt, sh, shoulder_center - shoulder_tol, shoulder_center + shoulder_tol);
        ++constraint_rejects;
        continue;
      }
    }

    // Cost = actual joint movement (numeric distance after normalisation, not wrapped shortest
    // angle) + quadratic penalty for deviation from functional position.
    // Movement term (2×) dominates; functional penalty is a tiebreaker for canonical posture.
    double movement_cost = 0.0;
    double functional_penalty = 0.0;
    for (std::size_t i = 0; i < candidate_joint_values.size(); ++i) {
      movement_cost += std::abs(candidate_joint_values[i] - current_joint_values[i]);
      if (i < 6) {
        const double d = candidate_joint_values[i] - q_func[i];
        functional_penalty += w_func[i] * d * d;
      }
    }
    const double candidate_cost = (2.0 * movement_cost) + (0.3 * functional_penalty);

    // RCLCPP_INFO(LOGGER, "computeBestIK: attempt %d succeeded, cost=%.4f (best=%.4f).",
    //   attempt, candidate_cost, best_cost);

    if (candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best_joint_values = candidate_joint_values;
    }
  }

  RCLCPP_INFO(LOGGER,
    "computeBestIK: %d/15 IK solutions found, %d rejected for shoulder constraint. Best cost: %.4f",
    ik_successes, constraint_rejects, best_cost);

  if (best_joint_values.empty()) {
    RCLCPP_ERROR(LOGGER,
      "computeBestIK: No valid IK solution found for target pose "
      "(x=%.3f y=%.3f z=%.3f). Target may be out of reach or in collision.",
      target_pose.position.x, target_pose.position.y, target_pose.position.z);
  }

  return best_joint_values;
}

std::vector<moveit::planning_interface::MoveGroupInterface::Plan>WordleBotController::generateCandidatePlans(int num_attempts)
{
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  plans.reserve(static_cast<std::size_t>(std::max(num_attempts, 0)));

  RCLCPP_INFO(LOGGER, "generateCandidatePlans: generating %d plan attempts.", num_attempts);
  int successes = 0;
  for (int attempt = 0; attempt < num_attempts; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = move_group_.plan(plan);
    if (static_cast<bool>(result)) {
      ++successes;
      RCLCPP_DEBUG(LOGGER, "generateCandidatePlans: attempt %d succeeded (%zu waypoints).",
        attempt, plan.trajectory_.joint_trajectory.points.size());
      plans.push_back(plan);
    } else {
      RCLCPP_WARN(LOGGER, "generateCandidatePlans: attempt %d failed (error code %d).",
        attempt, result.val);
    }
  }
  RCLCPP_INFO(LOGGER, "generateCandidatePlans: %d/%d plans succeeded.", successes, num_attempts);

  return plans;
}

moveit::planning_interface::MoveGroupInterface::Plan WordleBotController::selectBestPlan(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans,
                                                                                         const std::vector<double> & q_start,
                                                                                         const std::vector<double> & q_goal)
{
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  double best_cost = std::numeric_limits<double>::infinity();

  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);
  if (joint_model_group == nullptr) {
    return best_plan;
  }

  const auto & variable_names = joint_model_group->getVariableNames();
  const auto & joint_models = joint_model_group->getActiveJointModels();
  const double direct_distance = configurationDistance(joint_model_group, q_start, q_goal);
  const int base_joint_index = jointNameIndex(variable_names, "shoulder_pan_joint");
  const int wrist_joint_index = jointNameIndex(variable_names, "wrist_3_joint");

  // Functional position penalty at the goal state: sum of |q_goal[i] - q_func[i]| per joint.
  // This steers plan selection toward paths that land in a comfortable canonical posture.
  static const double q_func[6] = {0.0, -2.3562, -1.5708, -0.7854, 0.0, 0.0};
  double functional_goal_penalty = 0.0;
  for (std::size_t i = 0; i < q_goal.size() && i < 6; ++i) {
    functional_goal_penalty += std::abs(q_goal[i] - q_func[i]);
  }

  std::size_t plan_index = 0;
  for (const auto & plan : plans) {
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    const auto & points = trajectory.points;
    if (points.empty()) {
      ++plan_index;
      continue;
    }

    std::vector<int> trajectory_indices;
    trajectory_indices.reserve(variable_names.size());
    bool missing_joint = false;

    for (const auto & variable_name : variable_names) {
      const int index = jointNameIndex(trajectory.joint_names, variable_name);
      if (index < 0) {
        missing_joint = true;
        break;
      }
      trajectory_indices.push_back(index);
    }

    if (missing_joint) {
      ++plan_index;
      continue;
    }

    double total_path_length = 0.0;
    double base_rotation = 0.0;
    double wrist_rotation = 0.0;

    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
      if (point_index == 0) {
        continue;
      }

      const auto & previous_point = points[point_index - 1];
      const auto & current_point  = points[point_index];
      double segment_length = 0.0;

      for (std::size_t joint_index = 0; joint_index < joint_models.size(); ++joint_index) {
        const double prev_pos = previous_point.positions[trajectory_indices[joint_index]];
        const double curr_pos = current_point.positions[trajectory_indices[joint_index]];
        const double delta = jointDistance(joint_models[joint_index], prev_pos, curr_pos);
        segment_length += delta;

        if (static_cast<int>(joint_index) == base_joint_index) {
          base_rotation += delta;
        }
        if (static_cast<int>(joint_index) == wrist_joint_index) {
          wrist_rotation += delta;
        }
      }

      total_path_length += segment_length;
    }

    // Soft cost: path length (primary) + base rotation (secondary) + wrist rotation (tertiary)
    // + functional goal penalty (quaternary). No hard rejections — the lowest-cost plan always
    // wins regardless of ratio or rotation thresholds.
    const double plan_cost =
      (2.0 * total_path_length) +
      (1.5 * base_rotation) +
      (0.5 * wrist_rotation) +
      (0.25 * functional_goal_penalty);

    RCLCPP_INFO(LOGGER,
      "selectBestPlan: plan %zu — path=%.3f direct=%.3f ratio=%.2fx base_rot=%.3f rad cost=%.3f -> %s",
      plan_index,
      total_path_length, direct_distance,
      direct_distance > 1e-6 ? total_path_length / direct_distance : 0.0,
      base_rotation, plan_cost,
      plan_cost < best_cost ? "best so far" : "worse");

    if (plan_cost < best_cost) {
      best_cost = plan_cost;
      best_plan = plan;
    }
    ++plan_index;
  }

  if (best_cost > 15.0) {
    RCLCPP_WARN(LOGGER,
      "selectBestPlan: best plan cost=%.3f is high — motion may not be fully optimal.", best_cost);
  }

  return best_plan;
}

bool WordleBotController::moveToTarget(const geometry_msgs::msg::Pose & target)
{
  RCLCPP_INFO(LOGGER, "Target pose:\n  pos  x=%.3f y=%.3f z=%.3f\n  quat x=%.3f y=%.3f z=%.3f w=%.3f",
    target.position.x, target.position.y, target.position.z,
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

  move_group_.setStartStateToCurrentState();

  moveit_msgs::msg::JointConstraint shoulder;
  shoulder.joint_name        = "shoulder_lift_joint";
  shoulder.position          = -M_PI / 2.0;
  shoulder.tolerance_above   = M_PI / 180.0 * 110.0;
  shoulder.tolerance_below   = M_PI / 180.0 * 110.0;
  shoulder.weight            = 1.0;

  // Keep wrist_3_joint within [-π, π] throughout the planned path.
  // The UR RTDE interface reports wrist_3 in this range; going outside it causes
  // a PATH_TOLERANCE_VIOLATED (2π position error) on the trajectory controller.
  moveit_msgs::msg::JointConstraint wrist3;
  wrist3.joint_name      = "wrist_3_joint";
  wrist3.position        = 0.0;
  wrist3.tolerance_above = M_PI;
  wrist3.tolerance_below = M_PI;
  wrist3.weight          = 1.0;

  moveit_msgs::msg::Constraints constraints;
  constraints.joint_constraints.push_back(shoulder);
  constraints.joint_constraints.push_back(wrist3);
  move_group_.setPathConstraints(constraints);

  visualisePlan(nullptr, "Planning");

  move_group_.setStartStateToCurrentState();
  current_state = move_group_.getCurrentState(2.0);  // 2s timeout for fresh state
  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);

  std::vector<double> q_start;
  if (current_state != nullptr && joint_model_group != nullptr) {
    current_state->copyJointGroupPositions(joint_model_group, q_start);

    // DEBUG: log current joint angles so we can verify state monitor is reading correctly
    const auto & jnames = joint_model_group->getVariableNames();
    RCLCPP_INFO(LOGGER, "Current joint state (%zu joints):", q_start.size());
    for (std::size_t i = 0; i < q_start.size() && i < jnames.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        jnames[i].c_str(), q_start[i], q_start[i] * 180.0 / M_PI);
    }

    // DEBUG: forward kinematics — does the computed EEF position match where the robot actually is?
    const Eigen::Isometry3d & eef_tf = current_state->getGlobalLinkTransform("tool0");
    RCLCPP_INFO(LOGGER, "Current EEF pose (FK): x=%.4f y=%.4f z=%.4f",
      eef_tf.translation().x(), eef_tf.translation().y(), eef_tf.translation().z());
  } else {
    RCLCPP_ERROR(LOGGER, "getCurrentState returned null — state monitor has no data yet!");
  }

  const std::vector<double> best_q = computeBestIK(current_state, target);
  if (best_q.empty()) {
    move_group_.clearPathConstraints();
    visualisePlan(nullptr, "Planning Failed!");
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    return false;
  }

  // --- Diagnostic: log the chosen joint target values ---
  if (joint_model_group != nullptr) {
    const auto & joint_names = joint_model_group->getVariableNames();
    RCLCPP_INFO(LOGGER, "Target joint values (%zu joints):", best_q.size());
    for (std::size_t i = 0; i < best_q.size() && i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        joint_names[i].c_str(), best_q[i], best_q[i] * 180.0 / M_PI);
    }

    // --- Diagnostic: check shoulder constraint satisfaction at goal ---
    const int shoulder_idx = jointNameIndex(
      std::vector<std::string>(joint_names.begin(), joint_names.end()),
      "shoulder_lift_joint");
    if (shoulder_idx >= 0 && shoulder_idx < static_cast<int>(best_q.size())) {
      const double shoulder_val = best_q[shoulder_idx];
      const double shoulder_center = -M_PI / 2.0;
      const double shoulder_tol = M_PI / 180.0 * 110.0;
      const bool within = std::abs(shoulder_val - shoulder_center) <= shoulder_tol;
      RCLCPP_INFO(LOGGER,
        "Shoulder constraint check: shoulder_lift_joint=%.4f rad (%.1f deg), "
        "allowed range [%.4f, %.4f] -> %s",
        shoulder_val, shoulder_val * 180.0 / M_PI,
        shoulder_center - shoulder_tol, shoulder_center + shoulder_tol,
        within ? "WITHIN constraint" : "VIOLATES constraint");
    }

    // --- Diagnostic: check if goal joint values violate joint bounds ---
    if (current_state != nullptr) {
      moveit::core::RobotState goal_state(*current_state);
      goal_state.setJointGroupPositions(joint_model_group, best_q);
      goal_state.update();

      bool bounds_ok = true;
      const auto & active_joints = joint_model_group->getActiveJointModels();
      for (std::size_t i = 0; i < active_joints.size() && i < best_q.size(); ++i) {
        if (!active_joints[i]->satisfiesPositionBounds(&best_q[i])) {
          RCLCPP_ERROR(LOGGER,
            "Goal joint '%s' value %.4f rad VIOLATES joint limits!",
            active_joints[i]->getName().c_str(), best_q[i]);
          bounds_ok = false;
        }
      }
      if (bounds_ok) {
        RCLCPP_INFO(LOGGER, "Goal state joint bounds check: ALL within limits");
      }
    }
  }

  // --- Diagnostic: log the path constraint being applied ---
  RCLCPP_INFO(LOGGER,
    "Path constraint: shoulder_lift_joint center=%.4f rad, tolerance=+/-%.4f rad (%.1f deg)",
    -M_PI / 2.0, M_PI / 180.0 * 110.0, 110.0);

  move_group_.clearPoseTargets();
  move_group_.setJointValueTarget(best_q);

  const auto plans = generateCandidatePlans(5);
  move_group_.clearPathConstraints();

  if (plans.empty()) {
    visualisePlan(nullptr, "Planning Failed!");
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    return false;
  }

  auto plan = selectBestPlan(plans, q_start, best_q);
  const bool success = !plan.trajectory_.joint_trajectory.points.empty();

  if (success) {
    const double total_disp = computeTotalJointDisplacement(plan);
    const double direct_disp = configurationDistance(
      move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup), q_start, best_q);
    RCLCPP_INFO(LOGGER,
      "Selected plan: total_joint_disp=%.4f rad  direct_joint_disp=%.4f rad  ratio=%.3fx",
      total_disp, direct_disp, direct_disp > 1e-6 ? total_disp / direct_disp : 0.0);

    visualisePlan(&plan, "Executing");
    const auto exec_result = move_group_.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Motion executed successfully.");
    } else {
      RCLCPP_ERROR(LOGGER, "Execution FAILED with error code: %d", exec_result.val);
    }
  }
  else
  {
    visualisePlan(nullptr, "Planning Failed!");
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  return success;
}