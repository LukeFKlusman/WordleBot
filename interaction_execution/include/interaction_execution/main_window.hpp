#pragma once

#include <QList>
#include <QMainWindow>
#include <QStringList>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

class QAbstractAnimation;
class QEvent;
class QLabel;
class QMoveEvent;
class QPlainTextEdit;
class QProcess;
class QPushButton;
class QResizeEvent;
class QScrollArea;
class QVBoxLayout;
class CameraView;
class HlDigitalTwinView;
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

  enum class WordleGameMode
  {
    Unset,
    ModeA,
    ModeB
  };

  enum class ActiveView
  {
    SimView,
    TaskView,
    CameraView
  };

  enum class CameraMode
  {
    Raw,
    ComputerVision
  };

  struct MissionStepWidgets
  {
    QWidget * item{nullptr};
    QLabel * title{nullptr};
    QLabel * detail{nullptr};
  };

  bool eventFilter(QObject * watched, QEvent * event) override;
  void setupDrawer();
  void setupContentStack();
  void setupCameraPage();
  void setupVisualDesign();
  void setupDiagnosticsWindow();
  void setupVoiceControls();
  void setupSafetyControls();
  void setupGamificationBridge();
  void setupVoiceHelper();
  void setupHelpDialog();
  void loadWordleDictionary();
  void reserveSidebarWidth();
  void appendDiagnosticsEvent(const QString & message);
  void launchDiagnosticsWindow();
  void refreshDiagnosticsPanel();
  void renderMissionProgress(const QString & payload);
  void applyCoordinatorMissionState(const QString & state, bool log_event);
  SafetyControlMode safetyModeForCoordinatorState(const QString & state) const;
  void toggleDrawer();
  void switchToView(ActiveView view);
  void switchCameraMode(CameraMode mode);
  void updateDrawerActiveState();
  void updateDrawerLabelsVisibility();
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
  void publishMissionSignal(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher,
    const char * topic_name);
  void publishScanAndSweep();
  void publishGamificationFeedback(const QString & feedback);
  void publishGamificationMode(const QString & mode);
  void publishGamificationSecretWord(const QString & word);
  void publishGamificationPlayerGuess(const QString & guess);
  void resetGamificationGame();
  void beginVoiceRecording(bool reset_retry_sequence);
  void stopVoiceRecording();
  void promptManualVoiceOverride();
  bool isValidWordleWord(const QString & word) const;
  void updateSafetyBanner(const QString & text, const QString & color_hex);
  void moveEvent(QMoveEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;

  std::unique_ptr<Ui::MainWindow> ui_;
  rclcpp::Node::SharedPtr node_;
  static std::mutex rviz_initialization_mutex_;
  CameraView * camera_view_{nullptr};
  HlDigitalTwinView * hl_digital_twin_view_{nullptr};
  RvizSimView * rviz_view_{nullptr};
  WordleView * wordle_view_{nullptr};
  QWidget * diagnostics_window_{nullptr};

  // Drawer navigation
  QWidget * drawer_panel_{nullptr};
  bool drawer_expanded_{true};
  QPushButton * nav_sim_btn_{nullptr};
  QPushButton * nav_task_btn_{nullptr};
  QPushButton * nav_camera_btn_{nullptr};
  QPushButton * help_btn_{nullptr};
  QPushButton * diag_btn_{nullptr};
  QList<QLabel*> drawer_section_labels_;

  // Camera mode toggle
  QPushButton * cam_raw_btn_{nullptr};
  QPushButton * cam_cv_btn_{nullptr};
  ActiveView active_view_{ActiveView::SimView};
  CameraMode camera_mode_{CameraMode::Raw};

  // Diagnostics window (includes mission visualization now)
  QLabel * diagnostics_mission_value_label_{nullptr};
  QLabel * diagnostics_safety_value_label_{nullptr};
  QLabel * diagnostics_perception_value_label_{nullptr};
  QLabel * diagnostics_wordle_value_label_{nullptr};
  QPlainTextEdit * diagnostics_event_log_{nullptr};
  QPlainTextEdit * diagnostics_mission_json_view_{nullptr};
  QPlainTextEdit * diagnostics_game_json_view_{nullptr};

  // Mission visualization (now integrated into diagnostics)
  QScrollArea * diag_mission_scroll_{nullptr};
  QLabel * diag_mission_title_{nullptr};
  QLabel * diag_mission_summary_{nullptr};
  QWidget * diag_mission_steps_content_{nullptr};
  QVBoxLayout * diag_mission_steps_layout_{nullptr};
  QList<MissionStepWidgets> mission_step_widgets_;
  QString last_mission_progress_payload_;
  SafetyControlMode safety_mode_{SafetyControlMode::Idle};
  QStringList wordle_dictionary_;
  QString coordinator_mission_state_{QStringLiteral("IDLE")};
  QString current_robot_state_{QStringLiteral("IDLE")};
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
  bool voice_result_pending_{false};
  bool human_detected_{false};
  bool scan_game_board_active_{false};
  int voice_retry_rejections_{0};
  WordleGameMode current_wordle_mode_{WordleGameMode::Unset};
  QString pending_voice_guess_;
  QString voice_preview_text_{QStringLiteral("Awaiting input...")};
  QString voice_helper_stdout_buffer_;
  QString wordle_dictionary_path_;
  QProcess * voice_helper_process_{nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_and_sweep_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gamification_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gamification_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gamification_secret_word_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gamification_player_guess_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_detections_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gamification_guess_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gamification_diagnostics_sub_;
};
