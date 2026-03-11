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
  void updateFrame();
  void updatePixmap();
  void showStatusMessage(const QString & message);

  QLabel * image_label_{nullptr};
  QTimer frame_timer_;
  cv::VideoCapture capture_;
  QPixmap current_pixmap_;
};
