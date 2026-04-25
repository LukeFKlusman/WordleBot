#pragma once

#include <QTimer>
#include <QWidget>
#include <QShowEvent>

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>

#include <std_msgs/msg/string.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>

class RvizSimView : public QWidget
{
  Q_OBJECT

public:
  explicit RvizSimView(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);
  ~RvizSimView() override;

private:
  void showEvent(QShowEvent * event) override;
  void buildUi();
  void initializeRvizPanel();
  void createDisplays();
  void createDisplaysIfReady();
  void publishRelay();
  void ensureRobotDescriptionAvailable();
  void configureRobotModelDisplay(rviz_common::Display * display);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> robot_description_param_client_;

  rviz_common::RenderPanel * render_panel_{nullptr};
  std::shared_ptr<rviz_common::VisualizationManager> manager_;
  rviz_common::Display * robot_model_display_{nullptr};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    robot_desc_relay_pub_;

  // Cached URDF from either the original topic or the robot_state_publisher
  // parameter API. We keep re-publishing it so late RViz subscribers can still receive it.
  std::shared_ptr<std_msgs::msg::String> cached_robot_desc_;
  std::atomic<bool> robot_description_request_in_flight_{false};
  bool displays_created_{false};
  bool rviz_initialized_{false};

  QTimer ros_spin_timer_;
  QTimer robot_description_timer_;
};
