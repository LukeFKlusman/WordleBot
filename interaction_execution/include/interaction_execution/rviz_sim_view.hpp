#pragma once

#include <QTimer>
#include <QString>
#include <QWidget>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>

#include <std_msgs/msg/string.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>

class RvizSimView : public QWidget
{
  Q_OBJECT

public:
  explicit RvizSimView(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);
  ~RvizSimView() override;

private:
  void buildUi();
  void initializeRvizFrame();
  void createDisplays();
  void configureRobotModelDisplay(rviz_common::Display * display);
  QString resolveRvizConfigPath() const;
  void ensureRobotDescriptionTopic();
  void publishRobotDescription();
  void logRobotModelStatus();

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> robot_description_param_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_pub_;

  rviz_common::VisualizationFrame * frame_{nullptr};
  QWidget * render_widget_{nullptr};
  rviz_common::VisualizationManager * manager_{nullptr};
  rviz_common::Display * robot_model_display_{nullptr};
  std::string robot_description_xml_;
  bool robot_description_request_in_flight_{false};

  QTimer ros_spin_timer_;
  QTimer robot_description_timer_;
  QTimer robot_status_timer_;
  std::string last_status_;
};
