#pragma once

#include <QMainWindow>
#include <memory>

#include <rclcpp/rclcpp.hpp>

class CameraView;
class RvizSimView;

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);
  ~MainWindow() override;

private:
  void setupTabs();

  std::unique_ptr<Ui::MainWindow> ui_;
  rclcpp::Node::SharedPtr node_;
  CameraView * camera_view_{nullptr};
  RvizSimView * rviz_view_{nullptr};
};
