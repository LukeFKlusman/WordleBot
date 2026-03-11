#pragma once

#include <QString>
#include <QWidget>
#include <QTimer>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <rviz_common/render_panel.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

class RvizSimView : public QWidget
{
  Q_OBJECT
public:
  explicit RvizSimView(rclcpp::Node::SharedPtr node, QWidget* parent=nullptr);

private:
  void setupRviz();
  void addDisplays();
  QString resolveRvizConfigPath() const;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node_;

  rviz_common::VisualizationFrame* frame_{nullptr};
  QWidget* render_widget_{nullptr};
  rviz_common::VisualizationManager* viz_manager_{nullptr};

  QTimer spin_timer_;
};
