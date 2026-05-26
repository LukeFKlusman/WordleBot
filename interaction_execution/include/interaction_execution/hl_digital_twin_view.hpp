#pragma once

#include <QPointF>
#include <QString>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <hl_control/msg/gameboard_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <wordlebot_control/msg/pick_place_task.hpp>

#include <vector>

class QPainter;

class HlDigitalTwinView : public QWidget
{
  Q_OBJECT

public:
  explicit HlDigitalTwinView(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  struct LetterBlock
  {
    QString letter;
    QString object_id;
    QPointF position;
  };

  struct PickPlaceTask
  {
    QString object_id;
    QPointF pick;
    QPointF place;
  };

  void subscribe();
  void resetExecution();
  void drawHeader(QPainter & painter, const QRectF & rect) const;
  void drawWorkspace(QPainter & painter, const QRectF & rect) const;
  void drawTaskList(QPainter & painter, const QRectF & rect) const;
  QPointF gridToRobot(int row, int col) const;
  QPointF robotToCanvas(const QPointF & robot, const QRectF & rect) const;
  QString letterForObject(const QString & object_id) const;

  rclcpp::Node::SharedPtr node_;

  std::vector<LetterBlock> letters_;
  std::vector<PickPlaceTask> tasks_;
  QString target_word_;
  QString status_{QStringLiteral("Awaiting HL Control plan")};
  int completed_task_count_{0};
  bool mission_running_{false};
  int pulse_phase_{0};

  rclcpp::Subscription<hl_control::msg::GameboardState>::SharedPtr board_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr word_sub_;
  rclcpp::Subscription<wordlebot_control::msg::PickPlaceTask>::SharedPtr task_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_letters_sub_;

  QTimer animation_timer_;
};
