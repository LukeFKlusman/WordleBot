#pragma once

#include <QMainWindow>
#include <QStringList>
#include <memory>

#include <rclcpp/rclcpp.hpp>

class QResizeEvent;
class QMoveEvent;
class CameraView;
class RvizSimView;
class WordleView;

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);
  ~MainWindow() override;

private:
  void setupTabs();
  void setupVoiceControls();
  void updateVoiceControlsState();
  void setupMissionOverlay();
  void handleMainTabChanged(int index);
  void toggleMissionOverlay();
  void syncMissionOverlayGeometry();
  void updateMissionTabAppearance();
  void resizeEvent(QResizeEvent * event) override;
  void moveEvent(QMoveEvent * event) override;

  std::unique_ptr<Ui::MainWindow> ui_;
  rclcpp::Node::SharedPtr node_;
  CameraView * camera_view_{nullptr};
  RvizSimView * rviz_view_{nullptr};
  WordleView * wordle_view_{nullptr};
  QWidget * mission_overlay_{nullptr};
  QWidget * mission_tab_page_{nullptr};
  int mission_tab_index_{-1};
  int last_content_tab_index_{0};
  bool restoring_content_tab_{false};
  bool voice_recording_{false};
  QString pending_voice_guess_;
};
