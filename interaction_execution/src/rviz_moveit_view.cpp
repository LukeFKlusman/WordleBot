#include "interaction_execution/rviz_moveit_view.hpp"

#include <QApplication>
#include <QSizePolicy>
#include <QVBoxLayout>

#include <exception>

#include <rclcpp/rclcpp.hpp>

#include <QColor>

#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

namespace
{
constexpr const char * kRobotModelClassId = "rviz_default_plugins/RobotModel";
constexpr const char * kMoveCameraToolClassId = "rviz_default_plugins/MoveCamera";
constexpr const char * kOrbitViewClassId = "rviz_default_plugins/Orbit";
constexpr const char * kRobotDescriptionTopic = "/robot_description_moveit_relay";

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
}

RvizMoveItView::RvizMoveItView(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node,
  QWidget * parent)
: QWidget(parent), node_(std::move(node)), rviz_node_(std::move(rviz_node))
{
  buildUi();

  robot_desc_relay_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/robot_description_moveit_relay",
    rclcpp::QoS(1).reliable().transient_local());

  robot_desc_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/robot_description",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr || msg->data.empty()) {
        return;
      }

      cached_robot_desc_ = msg;
      if (robot_desc_relay_pub_) {
        robot_desc_relay_pub_->publish(*msg);
      }
      createDisplaysIfReady();
      if (manager_) {
        manager_->queueRender();
      }
    });

  connect(&robot_description_timer_, &QTimer::timeout, this, [this]() {
    if (cached_robot_desc_ && robot_desc_relay_pub_) {
      robot_desc_relay_pub_->publish(*cached_robot_desc_);
    }
    createDisplaysIfReady();
  });
  robot_description_timer_.start(1000);
}

RvizMoveItView::~RvizMoveItView()
{
  robot_description_timer_.stop();

  if (manager_) {
    manager_->stopUpdate();
    manager_.reset();
  }
  if (render_panel_) {
    render_panel_->deleteLater();
    render_panel_ = nullptr;
  }
}

void RvizMoveItView::showEvent(QShowEvent * event)
{
  QWidget::showEvent(event);

  if (manager_) {
    manager_->startUpdate();
    return;
  }

  if (rviz_initialized_) {
    return;
  }

  rviz_initialized_ = true;
  QTimer::singleShot(500, this, [this]() {
    initializeRvizPanel();
    createDisplaysIfReady();
  });
}

void RvizMoveItView::hideEvent(QHideEvent * event)
{
  QWidget::hideEvent(event);

  if (manager_) {
    manager_->stopUpdate();
  }
}

void RvizMoveItView::buildUi()
{
  setStyleSheet("RvizMoveItView { background-color: #0f151c; }");

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(12, 12, 12, 12);
}

void RvizMoveItView::initializeRvizPanel()
{
  if (!rviz_node_) {
    rviz_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
      "interaction_execution_rviz_moveit_panel");
  }

  render_panel_ = new rviz_common::RenderPanel(this);
  render_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  render_panel_->setAttribute(Qt::WA_NativeWindow, true);
  render_panel_->setFocusPolicy(Qt::StrongFocus);
  render_panel_->setMouseTracking(true);
  if (auto * layout = qobject_cast<QVBoxLayout *>(this->layout())) {
    layout->addWidget(render_panel_);
  }

  render_panel_->show();
  QApplication::processEvents();

  try {
    render_panel_->getRenderWindow()->initialize();

    auto clock = rviz_node_->get_raw_node()->get_clock();
    manager_ = std::make_shared<rviz_common::VisualizationManager>(
      render_panel_, rviz_node_, nullptr, clock);

    render_panel_->initialize(manager_.get());
    QApplication::processEvents();

    manager_->setFixedFrame("base_link");
    manager_->initialize();
    manager_->startUpdate();
    manager_->getViewManager()->setCurrentViewControllerType(kOrbitViewClassId);

    auto * tool_manager = manager_->getToolManager();
    rviz_common::Tool * move_camera_tool = nullptr;
    for (int i = 0; tool_manager != nullptr && i < tool_manager->numTools(); ++i) {
      auto * tool = tool_manager->getTool(i);
      if (tool != nullptr && tool->getClassId() == kMoveCameraToolClassId) {
        move_camera_tool = tool;
        break;
      }
    }

    if (tool_manager != nullptr && move_camera_tool == nullptr) {
      move_camera_tool = tool_manager->addTool(kMoveCameraToolClassId);
    }

    if (tool_manager != nullptr && move_camera_tool != nullptr) {
      tool_manager->setDefaultTool(move_camera_tool);
      tool_manager->setCurrentTool(move_camera_tool);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to activate RViz MoveCamera tool.");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "RViz initialization failed: %s", e.what());
    if (manager_) {
      manager_->stopUpdate();
      manager_.reset();
    }
    if (render_panel_) {
      render_panel_->deleteLater();
      render_panel_ = nullptr;
    }
    rviz_initialized_ = false;
  }
}

void RvizMoveItView::createDisplaysIfReady()
{
  if (!rviz_initialized_ || manager_ == nullptr || cached_robot_desc_ == nullptr || displays_created_) {
    return;
  }

  createDisplays();
}

void RvizMoveItView::createDisplays()
{
  if (!manager_) {
    RCLCPP_ERROR(node_->get_logger(), "RViz manager is null; cannot create displays.");
    return;
  }

  manager_->removeAllDisplays();
  robot_model_display_ = nullptr;

  auto * grid = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
  if (grid == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create RViz Grid display.");
  } else {
    if (auto * color = findPropertyByName(grid, "Color")) {
      color->setValue(QColor(Qt::gray));
    }
  }

  robot_model_display_ = manager_->createDisplay(kRobotModelClassId, "UR3e Robot", true);
  if (robot_model_display_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create RViz RobotModel display.");
    return;
  }

  configureRobotModelDisplay(robot_model_display_);

  if (auto * view = manager_->getViewManager()->getCurrent()) {
    if (auto * distance = findPropertyByName(view, "Distance")) {
      distance->setValue(2.5f);
    }
    if (auto * focal_point = findPropertyByName(view, "Focal Point")) {
      if (auto * x = findPropertyByName(focal_point, "X")) {
        x->setValue(0.0f);
      }
      if (auto * y = findPropertyByName(focal_point, "Y")) {
        y->setValue(0.0f);
      }
      if (auto * z = findPropertyByName(focal_point, "Z")) {
        z->setValue(0.5f);
      }
    }
    if (auto * pitch = findPropertyByName(view, "Pitch")) {
      pitch->setValue(0.3f);
    }
    if (auto * yaw = findPropertyByName(view, "Yaw")) {
      yaw->setValue(0.785f);
    }
  }

  render_panel_->setFocus(Qt::OtherFocusReason);
  manager_->queueRender();
  QApplication::processEvents();
  displays_created_ = true;
}

void RvizMoveItView::configureRobotModelDisplay(rviz_common::Display * display)
{
  if (display == nullptr || display->getClassId() != kRobotModelClassId) {
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
}
