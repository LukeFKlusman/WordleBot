#pragma once

#include <QLabel>
#include <QPixmap>
#include <QWidget>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class QResizeEvent;

class CameraView : public QWidget
{
  Q_OBJECT

public:
  explicit CameraView(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);
  ~CameraView() override;

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  void handleImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void updatePixmap();
  void showStatusMessage(const QString & message);

  rclcpp::Node::SharedPtr node_;
  QLabel * image_label_{nullptr};
  QPixmap current_pixmap_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
