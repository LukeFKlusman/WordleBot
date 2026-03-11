#include "interaction_execution/camera_view.hpp"

#include <QImage>
#include <QResizeEvent>
#include <QVBoxLayout>

#include <opencv2/imgproc.hpp>

CameraView::CameraView(QWidget * parent)
: QWidget(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);

  image_label_ = new QLabel("Camera feed will appear here", this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(320, 240);
  layout->addWidget(image_label_);

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
}

void CameraView::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updatePixmap();
}

void CameraView::updateFrame()
{
  if (!capture_.isOpened()) {
    return;
  }

  cv::Mat frame_bgr;
  if (!capture_.read(frame_bgr) || frame_bgr.empty()) {
    showStatusMessage("Camera frame read failed.");
    return;
  }

  cv::Mat frame_rgb;
  cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);

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
