#pragma once

#include <QLabel>
#include <QTimer>
#include <QWidget>

#include <opencv2/videoio.hpp>

class CameraView : public QWidget
{
  Q_OBJECT

public:
  explicit CameraView(QWidget * parent = nullptr);
  ~CameraView() override;

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
<<<<<<< Updated upstream
  void updateFrame();
=======
>>>>>>> Stashed changes
  void updatePixmap();
  void showStatusMessage(const QString & message);
  void handleImage(const sensor_msgs::msg::Image::SharedPtr msg);

<<<<<<< Updated upstream
=======
  rclcpp::Node::SharedPtr node_;
>>>>>>> Stashed changes
  QLabel * image_label_{nullptr};
  QTimer frame_timer_;
  cv::VideoCapture capture_;
  QPixmap current_pixmap_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
