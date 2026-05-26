#include "interaction_execution/hl_digital_twin_view.hpp"

#include <QFont>
#include <QFontMetrics>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace
{
constexpr int kGridCols = 13;
constexpr int kGridRows = 7;
constexpr double kGridStep = 0.075;
constexpr double kWorkspaceXMin = -0.45;
constexpr double kWorkspaceXMax = 0.45;
constexpr double kWorkspaceYMin = 0.0;
constexpr double kWorkspaceYMax = 0.45;
constexpr double kPi = 3.14159265358979323846;
constexpr int kWordleRow = 3;
constexpr int kWordleColStart = 4;
constexpr int kWordleColEnd = 8;

QColor withAlpha(const QColor & color, int alpha)
{
  QColor copy = color;
  copy.setAlpha(alpha);
  return copy;
}

bool isWordleCell(int row, int col)
{
  return row == kWordleRow && col >= kWordleColStart && col <= kWordleColEnd;
}

bool isForbiddenStagingCell(int row, int col)
{
  return row >= 0 && row <= 4 && col >= 3 && col <= 9 && !isWordleCell(row, col);
}

void drawArrow(QPainter & painter, const QPointF & from, const QPointF & to, const QColor & color, qreal width)
{
  QPen pen(color, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  painter.setPen(pen);
  painter.drawLine(from, to);

  const QLineF line(from, to);
  if (line.length() < 1.0) {
    return;
  }

  const double angle = std::atan2(-(to.y() - from.y()), to.x() - from.x());
  const double arrow_size = 9.0;
  const QPointF arrow_p1 = to + QPointF(
    std::sin(angle - kPi / 3.0) * arrow_size,
    std::cos(angle - kPi / 3.0) * arrow_size);
  const QPointF arrow_p2 = to + QPointF(
    std::sin(angle - kPi + kPi / 3.0) * arrow_size,
    std::cos(angle - kPi + kPi / 3.0) * arrow_size);

  QPainterPath head;
  head.moveTo(to);
  head.lineTo(arrow_p1);
  head.lineTo(arrow_p2);
  head.closeSubpath();
  painter.setPen(Qt::NoPen);
  painter.setBrush(color);
  painter.drawPath(head);
}
}  // namespace

HlDigitalTwinView::HlDigitalTwinView(rclcpp::Node::SharedPtr node, QWidget * parent)
: QWidget(parent), node_(std::move(node))
{
  setMinimumSize(720, 520);
  setAutoFillBackground(false);
  subscribe();

  connect(&animation_timer_, &QTimer::timeout, this, [this]() {
    pulse_phase_ = (pulse_phase_ + 1) % 120;
    update();
  });
  animation_timer_.start(50);
}

void HlDigitalTwinView::subscribe()
{
  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

  board_sub_ = node_->create_subscription<hl_control::msg::GameboardState>(
    "/perception/gameboard_state",
    latched_qos,
    [this](const hl_control::msg::GameboardState::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      letters_.clear();
      for (const auto & letter : msg->letters) {
        letters_.push_back({
          QString::fromStdString(letter.letter).trimmed().toUpper(),
          QString::fromStdString(letter.object_id).trimmed(),
          QPointF(letter.pose.pose.position.x, letter.pose.pose.position.y),
        });
      }

      if (tasks_.empty()) {
        completed_task_count_ = 0;
        mission_running_ = false;
        status_ = tr("Board received: waiting for HL Control plan");
      }
      update();
    });

  word_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/hl_control/word_request",
    latched_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      target_word_ = QString::fromStdString(msg->data).trimmed().toUpper();
      update();
    });

  task_sub_ = node_->create_subscription<wordlebot_control::msg::PickPlaceTask>(
    "/perception/letter_objects",
    10,
    [this](const wordlebot_control::msg::PickPlaceTask::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      tasks_.push_back({
        QString::fromStdString(msg->object_id).trimmed(),
        QPointF(msg->pick_pose.pose.position.x, msg->pick_pose.pose.position.y),
        QPointF(msg->place_pose.position.x, msg->place_pose.position.y),
      });
      completed_task_count_ = std::min(completed_task_count_, static_cast<int>(tasks_.size()));
      status_ = tr("HL Control plan queued: %1 pick/place step(s)").arg(tasks_.size());
      update();
    });

  start_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/start_mission",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg != nullptr && msg->data) {
        mission_running_ = true;
        completed_task_count_ = 0;
        status_ = tr("MoveIt execution running");
        update();
      }
    });

  goal_reached_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/goal_reached",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg != nullptr && msg->data) {
        completed_task_count_ = std::min(
          completed_task_count_ + 1,
          static_cast<int>(tasks_.size()));
        status_ = tr("MoveIt execution: %1/%2 step(s) complete")
          .arg(completed_task_count_)
          .arg(tasks_.size());
        update();
      }
    });

  mission_complete_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/mission_complete",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg != nullptr && msg->data) {
        completed_task_count_ = static_cast<int>(tasks_.size());
        mission_running_ = false;
        status_ = tr("Pick/place mission complete");
        update();
      }
    });

  clear_letters_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/clear_letter_objects",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg != nullptr && msg->data) {
        resetExecution();
      }
    });
}

void HlDigitalTwinView::resetExecution()
{
  tasks_.clear();
  completed_task_count_ = 0;
  mission_running_ = false;
  status_ = tr("Task queue cleared");
  update();
}

void HlDigitalTwinView::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor("#0f151c"));

  const QRectF bounds = rect().adjusted(18, 18, -18, -18);
  const QRectF header_rect(bounds.left(), bounds.top(), bounds.width(), 72);
  const QRectF body_rect(bounds.left(), header_rect.bottom() + 14, bounds.width(), bounds.height() - 86);

  const qreal list_width = std::clamp(bounds.width() * 0.27, 220.0, 320.0);
  const QRectF workspace_rect(
    body_rect.left(),
    body_rect.top(),
    body_rect.width() - list_width - 16,
    body_rect.height());
  const QRectF task_rect(
    workspace_rect.right() + 16,
    body_rect.top(),
    list_width,
    body_rect.height());

  drawHeader(painter, header_rect);
  drawWorkspace(painter, workspace_rect);
  drawTaskList(painter, task_rect);
}

void HlDigitalTwinView::drawHeader(QPainter & painter, const QRectF & rect) const
{
  QLinearGradient gradient(rect.topLeft(), rect.topRight());
  gradient.setColorAt(0.0, QColor("#13202c"));
  gradient.setColorAt(1.0, QColor("#111827"));
  painter.setPen(QPen(QColor("#263445"), 1));
  painter.setBrush(gradient);
  painter.drawRoundedRect(rect, 8, 8);

  painter.setPen(QColor("#f8fafc"));
  QFont title_font = painter.font();
  title_font.setPointSize(16);
  title_font.setBold(true);
  painter.setFont(title_font);
  painter.drawText(rect.adjusted(18, 10, -18, -36), Qt::AlignLeft | Qt::AlignVCenter,
    tr("HL Control Digital Twin"));

  QFont status_font = painter.font();
  status_font.setPointSize(9);
  status_font.setBold(false);
  painter.setFont(status_font);
  painter.setPen(QColor("#9fb0c3"));
  const QString word = target_word_.isEmpty() ? tr("Target: awaiting word") : tr("Target: %1").arg(target_word_);
  painter.drawText(rect.adjusted(18, 38, -18, -10), Qt::AlignLeft | Qt::AlignVCenter,
    tr("%1   |   %2").arg(word, status_));

  const int total = static_cast<int>(tasks_.size());
  const QString progress = total == 0
    ? tr("No plan")
    : tr("%1/%2 complete").arg(completed_task_count_).arg(total);
  const QRectF badge(rect.right() - 154, rect.top() + 20, 132, 32);
  painter.setBrush(withAlpha(mission_running_ ? QColor("#1f6feb") : QColor("#238636"), total == 0 ? 70 : 155));
  painter.setPen(QPen(withAlpha(QColor("#79c0ff"), 130), 1));
  painter.drawRoundedRect(badge, 6, 6);
  painter.setPen(QColor("#e6edf3"));
  painter.drawText(badge, Qt::AlignCenter, progress);
}

void HlDigitalTwinView::drawWorkspace(QPainter & painter, const QRectF & rect) const
{
  painter.setPen(QPen(QColor("#263445"), 1));
  painter.setBrush(QColor("#101923"));
  painter.drawRoundedRect(rect, 8, 8);

  const QRectF plot = rect.adjusted(30, 22, -30, -38);
  painter.setPen(QPen(QColor("#2f3f52"), 1));

  for (int row = 0; row < kGridRows; ++row) {
    for (int col = 0; col < kGridCols; ++col) {
      const QPointF center = robotToCanvas(gridToRobot(row, col), plot);
      const QRectF cell(center.x() - 13, center.y() - 13, 26, 26);
      if (isWordleCell(row, col)) {
        painter.setBrush(QColor("#f2cc60"));
        painter.setPen(QPen(QColor("#d29922"), 1.5));
        painter.drawRoundedRect(cell.adjusted(-2, -2, 2, 2), 5, 5);

        const int word_index = col - kWordleColStart;
        painter.setPen(QColor("#111827"));
        QFont font = painter.font();
        font.setBold(true);
        font.setPointSize(9);
        painter.setFont(font);
        painter.drawText(cell, Qt::AlignCenter,
          target_word_.size() == 5 ? target_word_.mid(word_index, 1) : QStringLiteral("_"));
      } else {
        painter.setBrush(isForbiddenStagingCell(row, col) ? QColor("#3a1f28") : QColor("#172231"));
        painter.setPen(QPen(isForbiddenStagingCell(row, col) ? QColor("#7f3344") : QColor("#314154"), 1));
        painter.drawEllipse(center, 4.0, 4.0);
      }
    }
  }

  for (int i = 0; i < static_cast<int>(tasks_.size()); ++i) {
    const auto & task = tasks_[i];
    const bool done = i < completed_task_count_;
    const bool active = mission_running_ && i == completed_task_count_;
    const QColor color = done ? QColor("#3fb950") : (active ? QColor("#79c0ff") : QColor("#f97316"));
    const int alpha = done ? 90 : (active ? 220 : 120);
    const qreal width = active ? 3.4 : 2.0;
    drawArrow(painter, robotToCanvas(task.pick, plot), robotToCanvas(task.place, plot), withAlpha(color, alpha), width);
  }

  std::map<QString, QPointF> object_positions;
  for (const auto & letter : letters_) {
    object_positions[letter.object_id] = letter.position;
  }
  for (int i = 0; i < completed_task_count_ && i < static_cast<int>(tasks_.size()); ++i) {
    object_positions[tasks_[i].object_id] = tasks_[i].place;
  }

  for (const auto & entry : object_positions) {
    const QString object_id = entry.first;
    const QPointF center = robotToCanvas(entry.second, plot);
    const bool active =
      mission_running_ &&
      completed_task_count_ < static_cast<int>(tasks_.size()) &&
      tasks_[completed_task_count_].object_id == object_id;
    const qreal pulse = active ? 2.0 + std::sin(pulse_phase_ / 120.0 * 2.0 * kPi) * 2.0 : 0.0;

    painter.setPen(QPen(active ? QColor("#a5d6ff") : QColor("#7dd3fc"), active ? 2.0 : 1.0));
    painter.setBrush(active ? QColor("#1f6feb") : QColor("#2563eb"));
    painter.drawEllipse(center, 14.0 + pulse, 14.0 + pulse);

    QFont font = painter.font();
    font.setBold(true);
    font.setPointSize(9);
    painter.setFont(font);
    painter.setPen(QColor("#ffffff"));
    painter.drawText(QRectF(center.x() - 14, center.y() - 14, 28, 28), Qt::AlignCenter,
      letterForObject(object_id));
  }

  painter.setPen(QColor("#8b949e"));
  QFont label_font = painter.font();
  label_font.setBold(false);
  label_font.setPointSize(8);
  painter.setFont(label_font);
  painter.drawText(rect.adjusted(18, rect.height() - 30, -18, -8), Qt::AlignLeft | Qt::AlignVCenter,
    tr("Robot workspace: 13 x 7 grid, pick blocks, Wordle slots, and HL Control pick/place path"));
}

void HlDigitalTwinView::drawTaskList(QPainter & painter, const QRectF & rect) const
{
  painter.setPen(QPen(QColor("#263445"), 1));
  painter.setBrush(QColor("#101923"));
  painter.drawRoundedRect(rect, 8, 8);

  painter.setPen(QColor("#f8fafc"));
  QFont title_font = painter.font();
  title_font.setPointSize(11);
  title_font.setBold(true);
  painter.setFont(title_font);
  painter.drawText(rect.adjusted(14, 12, -14, -rect.height() + 42), Qt::AlignLeft | Qt::AlignVCenter,
    tr("Pick and Place Plan"));

  QFont body_font = painter.font();
  body_font.setPointSize(8);
  body_font.setBold(false);
  painter.setFont(body_font);

  if (tasks_.empty()) {
    painter.setPen(QColor("#9fb0c3"));
    painter.drawText(rect.adjusted(14, 52, -14, -14), Qt::AlignTop | Qt::TextWordWrap,
      tr("Waiting for /perception/letter_objects from HL Control."));
    return;
  }

  qreal y = rect.top() + 52;
  const qreal row_height = 58;
  const int max_visible = std::max(1, static_cast<int>((rect.height() - 60) / row_height));
  const int first = std::max(0, std::min(completed_task_count_ - 1, static_cast<int>(tasks_.size()) - max_visible));
  const int last = std::min(static_cast<int>(tasks_.size()), first + max_visible);

  for (int i = first; i < last; ++i) {
    const auto & task = tasks_[i];
    const bool done = i < completed_task_count_;
    const bool active = mission_running_ && i == completed_task_count_;
    const QRectF item(rect.left() + 12, y, rect.width() - 24, row_height - 8);
    painter.setBrush(done ? QColor("#123524") : (active ? QColor("#122b45") : QColor("#172231")));
    painter.setPen(QPen(done ? QColor("#2ea043") : (active ? QColor("#1f6feb") : QColor("#314154")), 1));
    painter.drawRoundedRect(item, 6, 6);

    painter.setPen(done ? QColor("#7ee787") : (active ? QColor("#a5d6ff") : QColor("#d8dee9")));
    QFont item_title = painter.font();
    item_title.setBold(true);
    painter.setFont(item_title);
    painter.drawText(item.adjusted(10, 6, -10, -28), Qt::AlignLeft | Qt::AlignVCenter,
      tr("%1. %2").arg(i + 1).arg(letterForObject(task.object_id)));

    QFont detail_font = painter.font();
    detail_font.setBold(false);
    painter.setFont(detail_font);
    painter.setPen(QColor("#9fb0c3"));
    painter.drawText(item.adjusted(10, 27, -10, -6), Qt::AlignLeft | Qt::AlignVCenter,
      tr("(%1, %2) -> (%3, %4)")
        .arg(task.pick.x(), 0, 'f', 3)
        .arg(task.pick.y(), 0, 'f', 3)
        .arg(task.place.x(), 0, 'f', 3)
        .arg(task.place.y(), 0, 'f', 3));

    y += row_height;
  }
}

QPointF HlDigitalTwinView::gridToRobot(int row, int col) const
{
  return QPointF(kWorkspaceXMin + col * kGridStep, row * kGridStep);
}

QPointF HlDigitalTwinView::robotToCanvas(const QPointF & robot, const QRectF & rect) const
{
  const double x_ratio = (robot.x() - kWorkspaceXMin) / (kWorkspaceXMax - kWorkspaceXMin);
  const double y_ratio = (robot.y() - kWorkspaceYMin) / (kWorkspaceYMax - kWorkspaceYMin);
  return QPointF(
    rect.left() + x_ratio * rect.width(),
    rect.bottom() - y_ratio * rect.height());
}

QString HlDigitalTwinView::letterForObject(const QString & object_id) const
{
  for (const auto & letter : letters_) {
    if (letter.object_id == object_id && !letter.letter.isEmpty()) {
      return letter.letter.left(1);
    }
  }

  const QString trimmed = object_id.trimmed();
  if (!trimmed.isEmpty()) {
    return trimmed.left(1).toUpper();
  }

  return QStringLiteral("?");
}
