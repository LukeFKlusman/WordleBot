#pragma once

#include <QMainWindow>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class CameraView;
class RvizSimView;

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
<<<<<<< Updated upstream
=======
  void setupVoiceControls();
  void setupSafetyControls();
  void updateVoiceControlsState();
  void publishMissionState(const std::string & state);
  void publishMissionCommand(const std::string & command);
  void updateSafetyBanner(const QString & text, const QString & color_hex);
  void setupMissionOverlay();
  void handleMainTabChanged(int index);
  void toggleMissionOverlay();
  void syncMissionOverlayGeometry();
  void updateMissionTabAppearance();
  void resizeEvent(QResizeEvent * event) override;
  void moveEvent(QMoveEvent * event) override;
>>>>>>> Stashed changes

  std::unique_ptr<Ui::MainWindow> ui_;
  rclcpp::Node::SharedPtr node_;
  CameraView * camera_view_{nullptr};
  RvizSimView * rviz_view_{nullptr};
<<<<<<< Updated upstream
=======
  WordleView * wordle_view_{nullptr};
  QWidget * mission_overlay_{nullptr};
  QWidget * mission_tab_page_{nullptr};
  int mission_tab_index_{-1};
  int last_content_tab_index_{0};
  bool restoring_content_tab_{false};
  bool voice_recording_{false};
  bool human_detected_{false};
  QString pending_voice_guess_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_sub_;
>>>>>>> Stashed changes
};
