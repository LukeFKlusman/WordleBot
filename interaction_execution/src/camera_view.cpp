#include "interaction_execution/camera_view.hpp"

#include <QImage>
#include <QResizeEvent>
#include <QVBoxLayout>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <functional>

CameraView::CameraView(rclcpp::Node::SharedPtr node, QWidget * parent)
: QWidget(parent), node_(std::move(node))
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);

  image_label_ = new QLabel("Waiting for camera/image_raw...", this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(320, 240);
  layout->addWidget(image_label_);

  image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw",
    rclcpp::SensorDataQoS(),
    std::bind(&CameraView::handleImage, this, std::placeholders::_1));
}

CameraView::~CameraView()
{
  image_subscription_.reset();
}

void CameraView::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updatePixmap();
}

void CameraView::handleImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & ex) {
    showStatusMessage("Camera frame conversion failed.");
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "Failed to convert image from 'camera/image_raw': %s",
      ex.what());
    return;
  }

  cv::Mat frame_rgb;
  cv::cvtColor(cv_ptr->image, frame_rgb, cv::COLOR_BGR2RGB);

  const auto bytes_per_line = static_cast<int>(frame_rgb.step);
  QImage image(
    frame_rgb.data,
    frame_rgb.cols,
    frame_rgb.rows,
    bytes_per_line,
    QImage::Format_RGB888);

  current_pixmap_ = QPixmap::fromImage(image.copy());
  updatePixmap();
}

void CameraView::updatePixmap()
{
  if (current_pixmap_.isNull()) {
    return;
  }

  image_label_->setPixmap(
    current_pixmap_.scaled(
      image_label_->size(),
      Qt::KeepAspectRatio,
      Qt::SmoothTransformation));
}

void CameraView::showStatusMessage(const QString & message)
{
  current_pixmap_ = QPixmap();
  image_label_->setPixmap(QPixmap());
  image_label_->setText(message);
}
