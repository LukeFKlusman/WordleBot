#pragma once

#include <QList>
#include <QMainWindow>
#include <QStringList>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class QEvent;
class QLabel;
class QMoveEvent;
class QPlainTextEdit;
class QProcess;
class QResizeEvent;
class QVBoxLayout;
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
  enum class SafetyControlMode
  {
    Idle,
    Active,
    Stopped,
    Homing
  };

  struct MissionStepWidgets
  {
    QWidget * item{nullptr};
    QLabel * title{nullptr};
    QLabel * detail{nullptr};
  };

  bool eventFilter(QObject * watched, QEvent * event) override;
  void setupTabs();
  void setupDiagnosticsWindow();
  void setupVoiceControls();
  void setupSafetyControls();
  void setupGamificationBridge();
  void setupMissionOverlay();
  void setupVoiceHelper();
  void loadWordleDictionary();
  void reserveSidebarWidth();
  void appendDiagnosticsEvent(const QString & message);
  void handleMainTabChanged(int index);
  void launchDiagnosticsWindow();
  void refreshDiagnosticsPanel();
  void toggleMissionOverlay();
  void renderMissionProgress(const QString & payload);
  void updateDiagnosticsTabAppearance();
  void syncMissionOverlayGeometry();
  void updateMissionTabAppearance();
  void handleVoiceHelperStdout();
  void handleVoiceHelperStderr();
  void handleVoiceHelperMessage(const QString & payload);
  void sendVoiceHelperCommand(const QString & command);
  void sendVoiceHelperShutdown();
  void setVoicePreviewText(const QString & text);
  void resetVoicePreview(const QString & text = QStringLiteral("Awaiting input..."));
  void updateSafetyControlsState();
  void updateVoiceControlsState();
  void publishMissionState(const std::string & state);
  void publishMissionCommand(const std::string & command);
  void publishGamificationFeedback(const QString & feedback);
  void updateSafetyBanner(const QString & text, const QString & color_hex);
  void moveEvent(QMoveEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;

  std::unique_ptr<Ui::MainWindow> ui_;
  rclcpp::Node::SharedPtr node_;
  CameraView * camera_view_{nullptr};
  RvizSimView * rviz_view_{nullptr};
  WordleView * wordle_view_{nullptr};
  QWidget * diagnostics_window_{nullptr};
  QLabel * diagnostics_mission_value_label_{nullptr};
  QLabel * diagnostics_safety_value_label_{nullptr};
  QLabel * diagnostics_perception_value_label_{nullptr};
  QLabel * diagnostics_wordle_value_label_{nullptr};
  QPlainTextEdit * diagnostics_event_log_{nullptr};
  QPlainTextEdit * diagnostics_mission_json_view_{nullptr};
  QPlainTextEdit * diagnostics_game_json_view_{nullptr};
  QWidget * mission_overlay_{nullptr};
  QLabel * mission_title_label_{nullptr};
  QLabel * mission_summary_label_{nullptr};
  QWidget * mission_steps_content_{nullptr};
  QVBoxLayout * mission_steps_layout_{nullptr};
  QList<MissionStepWidgets> mission_step_widgets_;
  QString last_mission_progress_payload_;
  QWidget * mission_tab_page_{nullptr};
  int diagnostics_tab_index_{-1};
  int mission_tab_index_{-1};
  int last_content_tab_index_{0};
  SafetyControlMode safety_mode_{SafetyControlMode::Idle};
  QStringList wordle_dictionary_;
  QString coordinator_mission_state_{QStringLiteral("IDLE")};
  QString current_perception_state_{QStringLiteral("IDLE")};
  QString current_perception_status_{QStringLiteral("UNKNOWN")};
  QString current_wordle_status_{QStringLiteral("UNKNOWN")};
  QString current_wordle_guess_;
  int current_wordle_attempt_{0};
  int current_candidates_left_{0};
  int current_detection_count_{0};
  QString last_gamification_diagnostics_payload_;
  QStringList diagnostics_event_entries_;
  bool voice_recording_{false};
  bool voice_transcribing_{false};
  bool voice_helper_available_{false};
  bool human_detected_{false};
  QString pending_voice_guess_;
  QString voice_preview_text_{QStringLiteral("Awaiting input...")};
  QString voice_helper_stdout_buffer_;
  QString wordle_dictionary_path_;
  QProcess * voice_helper_process_{nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gamification_feedback_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_detections_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gamification_guess_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gamification_diagnostics_sub_;
};
