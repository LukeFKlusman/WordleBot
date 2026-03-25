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
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/yaml_config_reader.hpp>

namespace
{
constexpr const char * kRobotDescriptionTopic = "/robot_description";
constexpr const char * kRobotDescriptionParam = "robot_description";
constexpr const char * kRobotStatePublisherNode = "/robot_state_publisher";

rviz_common::properties::Property * findPropertyByName(
  rviz_common::properties::Property * parent,
  const QString & name)
{
  if (parent == nullptr) {
    return nullptr;
  }

  for (int i = 0; i < parent->numChildren(); ++i) {
    auto * child = parent->childAt(i);
    if (child != nullptr && child->getName() == name) {
      return child;
    }
  }

  return nullptr;
}

void hideRvizChrome(rviz_common::VisualizationFrame * frame)
{
  if (frame == nullptr) {
    return;
  }

  for (auto * dock_widget : frame->findChildren<QDockWidget *>()) {
    dock_widget->hide();
  }

  for (auto * tool_bar : frame->findChildren<QToolBar *>()) {
    tool_bar->hide();
  }

  if (frame->menuBar()) {
    frame->menuBar()->hide();
  }

  if (frame->statusBar()) {
    frame->statusBar()->hide();
  }
}

}  // namespace

RvizSimView::RvizSimView(rclcpp::Node::SharedPtr node, QWidget * parent)
: QWidget(parent), node_(std::move(node))
{
  buildUi();
  initializeRvizFrame();
  createDisplays();

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  robot_description_param_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(node_, kRobotStatePublisherNode);
  robot_description_pub_ = node_->create_publisher<std_msgs::msg::String>(
    kRobotDescriptionTopic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  connect(&ros_spin_timer_, &QTimer::timeout, this, [this]() {
    executor_->spin_some();
  });
  ros_spin_timer_.start(10);

  connect(&robot_description_timer_, &QTimer::timeout, this, [this]() {
    ensureRobotDescriptionTopic();
  });
  robot_description_timer_.start(1000);

  connect(&robot_status_timer_, &QTimer::timeout, this, [this]() {
    logRobotModelStatus();
  });
  robot_status_timer_.start(1000);

  ensureRobotDescriptionTopic();
}

RvizSimView::~RvizSimView()
{
  robot_status_timer_.stop();
  robot_description_timer_.stop();
  ros_spin_timer_.stop();
  if (executor_) {
    executor_->remove_node(node_);
  }
}

void RvizSimView::buildUi()
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
}

void RvizSimView::initializeRvizFrame()
{
  rviz_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
    "interaction_execution_rviz_panel");

  frame_ = new rviz_common::VisualizationFrame(rviz_node_, this);
  frame_->setApp(qobject_cast<QApplication *>(QApplication::instance()));
  frame_->setSplashPath("");
  frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  frame_->initialize(rviz_node_);

  render_widget_ = frame_->takeCentralWidget();
  if (render_widget_ != nullptr) {
    render_widget_->setParent(this);
    render_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    if (auto * layout = qobject_cast<QVBoxLayout *>(this->layout())) {
      layout->addWidget(render_widget_);
    }
  }

  hideRvizChrome(frame_);
  manager_ = frame_->getManager();
}

void RvizSimView::createDisplays()
{
  if (manager_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "RViz manager is null; cannot create displays.");
    return;
  }

  const auto rviz_config = resolveRvizConfigPath();
  if (!rviz_config.isEmpty()) {
    rviz_common::YamlConfigReader reader;
    rviz_common::Config config;
    reader.readFile(config, rviz_config);
    if (!reader.error()) {
      const auto viz_config = config.mapGetChild("Visualization Manager");
      if (viz_config.isValid()) {
        manager_->load(viz_config);
        RCLCPP_INFO(
          node_->get_logger(),
          "Loaded RViz config: %s",
          rviz_config.toStdString().c_str());
      }
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to load RViz config '%s': %s",
        rviz_config.toStdString().c_str(),
        reader.errorMessage().toStdString().c_str());
    }
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Could not locate ur_description RViz config; using built-in displays.");
  }

  auto * root = manager_->getRootDisplayGroup();
  if (root != nullptr) {
    for (int i = 0; i < root->numDisplays(); ++i) {
      auto * display = root->getDisplayAt(i);
      if (display != nullptr && display->getClassId() == "rviz_default_plugins/RobotModel") {
        robot_model_display_ = display;
        break;
      }
    }
  }

  if (robot_model_display_ == nullptr) {
    manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
    robot_model_display_ = manager_->createDisplay(
      "rviz_default_plugins/RobotModel", "UR3e Robot", true);
  }

  if (robot_model_display_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create RViz RobotModel display.");
    return;
  }

  manager_->setFixedFrame("base_link");
  configureRobotModelDisplay(robot_model_display_);
  manager_->queueRender();
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

void RvizSimView::configureRobotModelDisplay(rviz_common::Display * display)
{
  if (display == nullptr || display->getClassId() != "rviz_default_plugins/RobotModel") {
    return;
  }

  display->setEnabled(true);
  display->setValue(true);

  if (auto * alpha = findPropertyByName(display, "Alpha")) {
    alpha->setValue(1.0);
  }

  if (auto * visual_enabled = findPropertyByName(display, "Visual Enabled")) {
    visual_enabled->setValue(true);
  }

  if (auto * collision_enabled = findPropertyByName(display, "Collision Enabled")) {
    collision_enabled->setValue(false);
  }

  if (auto * source = findPropertyByName(display, "Description Source")) {
    source->setValue("Topic");
  }

  if (auto * topic = findPropertyByName(display, "Description Topic")) {
    if (auto * value = findPropertyByName(topic, "Value")) {
      value->setValue(kRobotDescriptionTopic);
    } else {
      topic->setValue(kRobotDescriptionTopic);
    }

    if (auto * durability = findPropertyByName(topic, "Durability Policy")) {
      durability->setValue("Transient Local");
    }

    if (auto * reliability = findPropertyByName(topic, "Reliability Policy")) {
      reliability->setValue("Reliable");
    }
  }

  if (auto * robot_description = findPropertyByName(display, "Robot Description")) {
    robot_description->setValue(kRobotDescriptionParam);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured RViz RobotModel display for topic '%s' and parameter '%s'.",
    kRobotDescriptionTopic,
    kRobotDescriptionParam);
}

void RvizSimView::ensureRobotDescriptionTopic()
{
  if (robot_description_param_client_ == nullptr || robot_description_request_in_flight_) {
    return;
  }

  if (!robot_description_param_client_->service_is_ready()) {
    return;
  }

  robot_description_request_in_flight_ = true;
  robot_description_param_client_->get_parameters(
    {kRobotDescriptionParam},
    [this](const std::shared_future<std::vector<rclcpp::Parameter>> future) {
      robot_description_request_in_flight_ = false;

      std::vector<rclcpp::Parameter> result;
      try {
        result = future.get();
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          5000,
          "Failed to query %s from %s: %s",
          kRobotDescriptionParam,
          kRobotStatePublisherNode,
          ex.what());
        return;
      }

      if (result.empty() || result.front().get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return;
      }

      const auto xml = result.front().as_string();
      if (xml.empty()) {
        return;
      }

      if (xml == robot_description_xml_) {
        return;
      }

      robot_description_xml_ = xml;
      publishRobotDescription();
      RCLCPP_INFO(
        node_->get_logger(),
        "Published %s from %s parameter (%zu bytes).",
        kRobotDescriptionTopic,
        kRobotStatePublisherNode,
        robot_description_xml_.size());
    });
}

void RvizSimView::publishRobotDescription()
{
  if (robot_description_pub_ == nullptr || robot_description_xml_.empty()) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = robot_description_xml_;
  robot_description_pub_->publish(msg);
}

void RvizSimView::logRobotModelStatus()
{
  if (robot_model_display_ == nullptr) {
    return;
  }

  auto * status_root = findPropertyByName(robot_model_display_, "Status");
  if (status_root == nullptr) {
    return;
  }

  std::string summary = "OK";
  for (int i = 0; i < status_root->numChildren(); ++i) {
    auto * child = status_root->childAt(i);
    auto * status = dynamic_cast<rviz_common::properties::StatusProperty *>(child);
    if (status == nullptr) {
      continue;
    }
    if (status->getLevel() == rviz_common::properties::StatusProperty::Ok) {
      continue;
    }

    summary = status->getName().toStdString() + ": " + status->getValue().toString().toStdString();
    break;
  }

  if (summary == last_status_) {
    return;
  }

  last_status_ = summary;
  if (summary == "OK") {
    RCLCPP_INFO(node_->get_logger(), "RobotModel status: OK");
  } else {
    RCLCPP_WARN(node_->get_logger(), "RobotModel status: %s", summary.c_str());
  }
}
