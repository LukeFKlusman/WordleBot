#include "interaction_execution/camera_view.hpp"

#include <QImage>
#include <QResizeEvent>
#include <QVBoxLayout>

#include <opencv2/imgproc.hpp>

<<<<<<< Updated upstream
CameraView::CameraView(QWidget * parent)
: QWidget(parent)
=======
namespace
{
constexpr const char * kCameraTopic = "/camera/camera/color/image_raw";
}

CameraView::CameraView(rclcpp::Node::SharedPtr node, QWidget * parent)
: QWidget(parent), node_(std::move(node))
>>>>>>> Stashed changes
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);

<<<<<<< Updated upstream
  image_label_ = new QLabel("Camera feed will appear here", this);
=======
  image_label_ = new QLabel("Waiting for /camera/camera/color/image_raw...", this);
>>>>>>> Stashed changes
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(320, 240);
  layout->addWidget(image_label_);

<<<<<<< Updated upstream
  if (!capture_.open(0, cv::CAP_ANY)) {
    showStatusMessage("Could not open laptop camera (index 0).");
    return;
  }

  connect(&frame_timer_, &QTimer::timeout, this, &CameraView::updateFrame);
  frame_timer_.start(33);
}

CameraView::~CameraView()
{
  frame_timer_.stop();
  if (capture_.isOpened()) {
    capture_.release();
  }
=======
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    kCameraTopic,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      handleImage(msg);
    });

  RCLCPP_INFO(node_->get_logger(), "Camera tab subscribed to %s.", kCameraTopic);
>>>>>>> Stashed changes
}

CameraView::~CameraView() = default;

void CameraView::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updatePixmap();
}

<<<<<<< Updated upstream
void CameraView::updateFrame()
{
  if (!capture_.isOpened()) {
    return;
  }

  cv::Mat frame_bgr;
  if (!capture_.read(frame_bgr) || frame_bgr.empty()) {
    showStatusMessage("Camera frame read failed.");
=======
void CameraView::handleImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  cv_bridge::CvImagePtr cv_image;
  try {
    cv_image = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const cv_bridge::Exception & ex) {
    showStatusMessage("Failed to decode ROS camera frame.");
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "Failed to convert image from %s: %s",
      kCameraTopic,
      ex.what());
>>>>>>> Stashed changes
    return;
  }

  cv::Mat frame_rgb;
<<<<<<< Updated upstream
  cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);
=======
  cv::cvtColor(cv_image->image, frame_rgb, cv::COLOR_BGR2RGB);
>>>>>>> Stashed changes

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

  image_label_->setText(QString());
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
