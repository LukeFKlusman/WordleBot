#include "wordleBot_control/wordle_bot_control_node.hpp"

#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotControlNode");

WordleBotControlNode::WordleBotControlNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("wordle_bot_control_node", options)),
  controller_(std::make_shared<WordleBotController>(node_))
{
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wordle_bot/goal_pose", 10,
    std::bind(&WordleBotControlNode::goalCallback, this, std::placeholders::_1));

  motion_complete_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/motion_complete", 10);

  RCLCPP_INFO(LOGGER, "WordleBotControlNode initialised.");
}

void WordleBotControlNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "Received goal on /wordle_bot/goal_pose — executing motion.");
  controller_->moveToTarget(msg->pose);

  std_msgs::msg::Bool complete;
  complete.data = true;
  motion_complete_pub_->publish(complete);
  RCLCPP_INFO(LOGGER, "Motion complete — published to /wordle_bot/motion_complete.");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
WordleBotControlNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void WordleBotControlNode::setupScene()
{
  controller_->clearCollisionScene();
  controller_->setupCollisionScene();
}

void WordleBotControlNode::run()
{
  RCLCPP_INFO(LOGGER, "Running goal sequence.");

  // Test goal: end-effector facing downward (180 deg roll), matching hello_ur3eMoveit
  auto target1 = WordleBotController::buildPose( 0.3, 0.25, 0.25,   // x, y, z
                                                M_PI, 0.0, 0.0 );    // roll, pitch, yaw

  auto target2 = WordleBotController::buildPose( -0.2, 0.3, 0.15, M_PI, 0.0, 0.0 ); 
  auto target3 = WordleBotController::buildPose( 0.2, 0.25, 0.20, M_PI, 0.0, 0.0 );
  auto target4 = WordleBotController::buildPose( -0.3, 0.3, 0.10, M_PI, 0.0, 0.0 );

  controller_->moveToTarget(target1);
  controller_->moveToTarget(target2);
  controller_->moveToTarget(target3);
  controller_->moveToTarget(target4);

  return;
}
