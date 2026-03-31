#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "wordleBot_control/wordle_bot_controller.hpp"


class WordleBotControlNode
{
public:
  explicit WordleBotControlNode(const rclcpp::NodeOptions & options);
  ~WordleBotControlNode();

  // Used by the executor in main to spin this node
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Delegate scene setup to the controller (collision objects, etc.)
  void setupScene();

  // Block main thread until shutdown; mission execution happens in missionThread_
  void run();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WordleBotController> controller_;

  // Topic-based goal interface
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_complete_pub_;

  // Mission state (extensible for TC1.4 Stop/Resume/Abort)
  enum class MissionState { IDLE, RUNNING };

  // Thread-safe goal queue
  std::queue<geometry_msgs::msg::Pose> goal_queue_;
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::atomic<MissionState> mission_state_{MissionState::IDLE};
  std::thread mission_thread_;

  // Enqueue incoming goal; returns immediately
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Runs on mission_thread_: dequeues and executes goals sequentially
  void missionLoop();
};
