#include "interaction_execution/rviz_sim_view.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QApplication>
#include <QDockWidget>
#include <QMenuBar>
#include <QSizePolicy>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include <rviz_common/config.hpp>
#include <rviz_common/yaml_config_reader.hpp>

RvizSimView::RvizSimView(rclcpp::Node::SharedPtr node, QWidget* parent)
: QWidget(parent), node_(std::move(node))
{
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0,0,0,0);

  exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  rviz_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("interaction_execution_rviz");

  setupRviz();
  if (render_widget_ != nullptr) {
    layout->addWidget(render_widget_);
  }
  addDisplays();

  // Spin ROS so RViz gets TF/joint_states updates
  connect(&spin_timer_, &QTimer::timeout, this, [this]() {
    exec_->spin_some();
  });
  spin_timer_.start(10); // 10–50ms is fine
}

void RvizSimView::setupRviz()
{
  frame_ = new rviz_common::VisualizationFrame(rviz_node_, this);
  frame_->setApp(qobject_cast<QApplication *>(QApplication::instance()));
  frame_->setSplashPath("");
  frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  frame_->initialize(rviz_node_);
  render_widget_ = frame_->takeCentralWidget();
  if (render_widget_ != nullptr) {
    render_widget_->setParent(this);
    render_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  for (auto * dock_widget : frame_->findChildren<QDockWidget *>()) {
    dock_widget->hide();
  }
  for (auto * tool_bar : frame_->findChildren<QToolBar *>()) {
    tool_bar->hide();
  }
  if (frame_->menuBar()) {
    frame_->menuBar()->hide();
  }
  if (frame_->statusBar()) {
    frame_->statusBar()->hide();
  }

  viz_manager_ = frame_->getManager();

  const auto rviz_config = resolveRvizConfigPath();
  if (rviz_config.isEmpty()) {
    RCLCPP_WARN(node_->get_logger(), "Could not locate UR RViz config. Using fallback displays.");
    return;
  }

  rviz_common::YamlConfigReader reader;
  rviz_common::Config config;
  reader.readFile(config, rviz_config);
  if (reader.error()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to load RViz config '%s': %s",
      rviz_config.toStdString().c_str(),
      reader.errorMessage().toStdString().c_str());
    return;
  }

  const auto viz_config = config.mapGetChild("Visualization Manager");
  if (!viz_config.isValid()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "RViz config '%s' does not contain a Visualization Manager section. Using fallback displays.",
      rviz_config.toStdString().c_str());
    return;
  }

  viz_manager_->load(viz_config);
}

void RvizSimView::addDisplays()
{
  if (viz_manager_->getRootDisplayGroup()->numDisplays() > 0) {
    return;
  }

  auto * robot =
    viz_manager_->createDisplay("rviz_default_plugins/RobotModel", "Robot Model", true);
  robot->subProp("Robot Description")->setValue("/robot_description");
  viz_manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
  viz_manager_->setFixedFrame("base_link");
}

QString RvizSimView::resolveRvizConfigPath() const
{
  try {
    const auto package_share = ament_index_cpp::get_package_share_directory("ur_description");
    return QString::fromStdString(package_share + "/rviz/view_robot.rviz");
  } catch (const std::exception &) {
    return QString();
  }
}
