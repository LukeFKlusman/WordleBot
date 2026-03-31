#include "wordleBot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <thread>

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

  mission_thread_ = std::thread(&WordleBotControlNode::missionLoop, this);

  // Wake the mission thread when the node shuts down so it can exit cleanly
  rclcpp::on_shutdown([this]() { cv_.notify_all(); });

  RCLCPP_INFO(LOGGER, "WordleBotControlNode initialised.");
}

WordleBotControlNode::~WordleBotControlNode()
{
  cv_.notify_all();
  if (mission_thread_.joinable()) {
    mission_thread_.join();
  }
}

void WordleBotControlNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    goal_queue_.push(msg->pose);
  }
  cv_.notify_one();
  RCLCPP_INFO(LOGGER, "Goal enqueued. Queue size: %zu", goal_queue_.size());
}

void WordleBotControlNode::missionLoop()
{
  RCLCPP_INFO(LOGGER, "Mission thread started.");
  while (rclcpp::ok()) {
    geometry_msgs::msg::Pose goal;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this]() {
        return !goal_queue_.empty() || !rclcpp::ok();
      });
      if (!rclcpp::ok()) break;
      goal = goal_queue_.front();
      goal_queue_.pop();
    }

    // Lock released — safe to call the long-running moveToTarget
    mission_state_ = MissionState::RUNNING;
    RCLCPP_INFO(LOGGER, "Mission thread: executing goal.");
    controller_->moveToTarget(goal);

    std_msgs::msg::Bool complete;
    complete.data = true;
    motion_complete_pub_->publish(complete);
    RCLCPP_INFO(LOGGER, "Motion complete published.");

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (goal_queue_.empty()) {
        mission_state_ = MissionState::IDLE;
      }
    }
  }
  RCLCPP_INFO(LOGGER, "Mission thread exiting.");
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
  RCLCPP_INFO(LOGGER, "run(): mission interface active — waiting for goals on /wordle_bot/goal_pose.");
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(LOGGER, "run(): shutdown detected, joining mission thread.");
  if (mission_thread_.joinable()) {
    mission_thread_.join();
  }
  RCLCPP_INFO(LOGGER, "run(): mission thread joined.");
}
