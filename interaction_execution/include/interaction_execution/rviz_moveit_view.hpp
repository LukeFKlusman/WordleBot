#pragma once

#include <QTimer>
#include <QWidget>
#include <QShowEvent>

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <std_msgs/msg/string.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>

class RvizMoveItView : public QWidget
{
  Q_OBJECT

public:
  explicit RvizMoveItView(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node,
    QWidget * parent = nullptr);
  ~RvizMoveItView() override;

private:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;
  void buildUi();
  void initializeRvizPanel();
  void createDisplays();
  void createDisplaysIfReady();
  void configureRobotModelDisplay(rviz_common::Display * display);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node_;

  rviz_common::RenderPanel * render_panel_{nullptr};
  std::shared_ptr<rviz_common::VisualizationManager> manager_;
  rviz_common::Display * robot_model_display_{nullptr};
  rviz_common::Display * planning_scene_display_{nullptr};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_desc_relay_pub_;
  std::shared_ptr<std_msgs::msg::String> cached_robot_desc_;

  bool displays_created_{false};
  bool rviz_initialized_{false};

  QTimer robot_description_timer_;
};
