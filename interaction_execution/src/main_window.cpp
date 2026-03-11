#include "interaction_execution/main_window.hpp"

#include "interaction_execution/camera_view.hpp"
#include "interaction_execution/rviz_sim_view.hpp"
#include "ui_rs2_concept.h"

#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent), ui_(std::make_unique<Ui::MainWindow>()), node_(std::move(node))
{
  ui_->setupUi(this);
  setupTabs();
}

MainWindow::~MainWindow() = default;

void MainWindow::setupTabs()
{
  auto * sim_layout = new QVBoxLayout(ui_->tab);
  sim_layout->setContentsMargins(0, 0, 0, 0);
  rviz_view_ = new RvizSimView(node_, ui_->tab);
  sim_layout->addWidget(rviz_view_);

  auto * camera_layout = new QVBoxLayout(ui_->tab_2);
  camera_layout->setContentsMargins(0, 0, 0, 0);
  camera_view_ = new CameraView(ui_->tab_2);
  camera_layout->addWidget(camera_view_);
}
