#include "interaction_execution/main_window.hpp"

#include "interaction_execution/camera_view.hpp"
#include "interaction_execution/hl_digital_twin_view.hpp"
#include "interaction_execution/rviz_sim_view.hpp"
#include "interaction_execution/wordle_view.hpp"
#include "ui_rs2_concept.h"

#include <algorithm>
#include <mutex>
#include <QAbstractAnimation>
#include <QFile>
#include <QFileInfo>
#include <QCloseEvent>
#include <QEasingCurve>
#include <QPropertyAnimation>
#include <QDateTime>
#include <QEvent>
#include <QFrame>
#include <QFontDatabase>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QLayoutItem>
#include <QMessageBox>
#include <QMouseEvent>
#include <QMoveEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QStyle>
#include <QTabBar>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
constexpr const char * kPerceptionStateTopic = "/mission/state";
constexpr const char * kCoordinatorMissionStateTopic = "/wordle_bot/mission_state";
constexpr const char * kRobotStateTopic = "/wordle_bot/robot_state";
constexpr const char * kMissionCompleteTopic = "/wordle_bot/mission_complete";
constexpr const char * kMissionProgressTopic = "/wordle_bot/mission_progress";
constexpr const char * kMissionCommandTopic = "/wordle_bot/mission_cmd";
constexpr const char * kStartMissionTopic = "/wordle_bot/start_mission";
constexpr const char * kResumeMissionTopic = "/wordle_bot/resume_mission";
constexpr const char * kStopMissionTopic = "/wordle_bot/stop_mission";
constexpr const char * kScanAndSweepTopic = "/wordle_bot/scan_and_sweep";
constexpr const char * kAbortMissionTopic = "/wordle_bot/abort_mission";
constexpr const char * kHumanDetectedTopic = "/perception/human_detected";
constexpr const char * kPerceptionStatusTopic = "/perception/status";
constexpr const char * kPerceptionDetectionsTopic = "/perception/detections";
constexpr const char * kGamificationGuessTopic = "/gamification/guess";
constexpr const char * kGamificationFeedbackTopic = "/gamification/feedback";
constexpr const char * kGamificationModeTopic = "/gamification/mode";
constexpr const char * kGamificationSecretWordTopic = "/gamification/secret_word";
constexpr const char * kGamificationPlayerGuessTopic = "/gamification/player_guess";
constexpr const char * kDiagnosticsTopic = "/diagnostics";

QIcon makeRecordIcon()
{
  QPixmap pixmap(18, 18);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor("#ef4444"));
  painter.drawEllipse(2, 2, 14, 14);

  return QIcon(pixmap);
}

QIcon makeStopIcon()
{
  QPixmap pixmap(18, 18);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor("#f8fafc"));
  painter.drawRect(3, 3, 12, 12);

  return QIcon(pixmap);
}

QIcon makeConfirmIcon()
{
  QPixmap pixmap(18, 18);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);
  QPen pen(QColor("#4ade80"));
  pen.setWidth(3);
  pen.setCapStyle(Qt::RoundCap);
  pen.setJoinStyle(Qt::RoundJoin);
  painter.setPen(pen);
  painter.drawLine(3, 10, 7, 14);
  painter.drawLine(7, 14, 15, 4);

  return QIcon(pixmap);
}

QIcon makeLaunchIcon()
{
  QPixmap pixmap(16, 16);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QPen pen(QColor("#cbd5e1"));
  pen.setWidth(2);
  pen.setCapStyle(Qt::SquareCap);
  pen.setJoinStyle(Qt::MiterJoin);
  painter.setPen(pen);
  painter.setBrush(Qt::NoBrush);

  painter.drawLine(3, 13, 3, 5);
  painter.drawLine(3, 13, 11, 13);
  painter.drawLine(13, 7, 13, 3);
  painter.drawLine(9, 3, 13, 3);
  painter.drawLine(7, 9, 13, 3);
  // painter.drawLine(10, 9, 13, 9);
  // painter.drawLine(10, 3, 10, 6);

  return QIcon(pixmap);
}

}  // namespace

std::mutex MainWindow::rviz_initialization_mutex_;

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent), ui_(std::make_unique<Ui::MainWindow>()), node_(std::move(node))
{
  ui_->setupUi(this);
  setupVisualDesign();
  loadWordleDictionary();
  setupDrawer();
  setupContentStack();
  setupGamificationBridge();
  setupVoiceControls();
  setupVoiceHelper();
  setupSafetyControls();
  reserveSidebarWidth();
}

MainWindow::~MainWindow()
{
  sendVoiceHelperShutdown();

  if (voice_helper_process_ != nullptr) {
    if (voice_helper_process_->state() != QProcess::NotRunning) {
      voice_helper_process_->terminate();
      if (!voice_helper_process_->waitForFinished(1000)) {
        voice_helper_process_->kill();
        voice_helper_process_->waitForFinished(1000);
      }
    }
  }
}

void MainWindow::loadWordleDictionary()
{
  QString dictionary_path;

  try {
    const QString interaction_share = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("interaction_execution"));
    dictionary_path = interaction_share + "/gamification/dictionary.txt";
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to locate interaction_execution share directory for dictionary lookup: %s",
      ex.what());
    return;
  }

  QFile dictionary_file(dictionary_path);
  if (!dictionary_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to open Wordle dictionary at %s",
      dictionary_path.toStdString().c_str());
    return;
  }

  QTextStream stream(&dictionary_file);
  QStringList loaded_words;
  while (!stream.atEnd()) {
    const QString word = stream.readLine().trimmed().toLower();
    if (word.size() == 5 && std::all_of(word.cbegin(), word.cend(), [](const QChar ch) {
          return ch.isLetter();
        }))
    {
      loaded_words.append(word);
    }
  }

  loaded_words.removeDuplicates();
  loaded_words.sort();
  wordle_dictionary_ = loaded_words;
  wordle_dictionary_path_ = dictionary_path;

  RCLCPP_INFO(
    node_->get_logger(),
    "Loaded %zu Wordle dictionary entries for GUI preview guesses.",
    static_cast<std::size_t>(wordle_dictionary_.size()));
}

bool MainWindow::eventFilter(QObject * watched, QEvent * event)
{
  (void)watched;
  (void)event;
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::setupDrawer()
{
  // Style drawer wrapper
  ui_->drawerWrapper->setStyleSheet(
    "QWidget#drawerWrapper {"
    "  background-color: #111820;"
    "  border-right: 1px solid rgba(139, 148, 158, 0.18);"
    "}");

  // Style hamburger button with better polish
  ui_->hamburgerButton->setStyleSheet(
    "QPushButton#hamburgerButton {"
    "  background-color: transparent;"
    "  color: #79c0ff;"
    "  border: 1px solid rgba(139, 148, 158, 0.3);"
    "  border-radius: 6px;"
    "  font-size: 14pt;"
    "  font-weight: bold;"
    "}"
    "QPushButton#hamburgerButton:hover {"
    "  background-color: rgba(121, 192, 255, 0.1);"
    "  color: #a5d6ff;"
    "}");

  connect(ui_->hamburgerButton, &QPushButton::clicked, this, &MainWindow::toggleDrawer);

  // Populate drawer panel with better structure
  auto * drawer_layout = new QVBoxLayout(ui_->drawerPanel);
  drawer_layout->setContentsMargins(12, 16, 12, 16);
  drawer_layout->setSpacing(20);

  // ===== VIEW SELECTOR SECTION =====
  auto * view_section_label = new QLabel(tr("VIEW SELECTOR"));
  view_section_label->setStyleSheet(
    "QLabel {"
    "  color: #79c0ff;"
    "  font-size: 8pt;"
    "  font-weight: 800;"
    "  letter-spacing: 2px;"
    "  margin-bottom: 4px;"
    "}");
  drawer_section_labels_.append(view_section_label);
  drawer_layout->addWidget(view_section_label);

  // Shared style function for drawer items
  auto setup_nav_button = [this, drawer_layout](
    QPushButton *& btn, const QString & text, const QString & emoji) {
    btn = new QPushButton(emoji + " " + text);
    btn->setCursor(Qt::PointingHandCursor);
    btn->setStyleSheet(
      "QPushButton {"
      "  background-color: transparent;"
      "  color: #8b949e;"
      "  border: none;"
      "  border-left: 3px solid transparent;"
      "  padding: 10px 12px;"
      "  text-align: left;"
      "  font-size: 11pt;"
      "  font-weight: 500;"
      "}"
      "QPushButton:hover {"
      "  background-color: rgba(121, 192, 255, 0.1);"
      "  color: #79c0ff;"
      "}"
      "QPushButton:pressed {"
      "  background-color: rgba(31, 111, 235, 0.2);"
      "}");
    drawer_layout->addWidget(btn);
  };

  setup_nav_button(nav_sim_btn_, tr("Sim View"), "📺");
  setup_nav_button(nav_task_btn_, tr("Task View"), "🤖");
  setup_nav_button(nav_camera_btn_, tr("Camera View"), "🎥");

  connect(nav_sim_btn_, &QPushButton::clicked, this, [this]() {
    switchToView(ActiveView::SimView);
  });
  connect(nav_task_btn_, &QPushButton::clicked, this, [this]() {
    switchToView(ActiveView::TaskView);
  });
  connect(nav_camera_btn_, &QPushButton::clicked, this, [this]() {
    switchToView(ActiveView::CameraView);
  });

  // ===== HELP & INFO SECTION =====
  // Spacer
  drawer_layout->addStretch();

  // Divider
  auto * divider = new QFrame(ui_->drawerPanel);
  divider->setFrameShape(QFrame::HLine);
  divider->setStyleSheet(
    "QFrame {"
    "  background-color: rgba(139, 148, 158, 0.18);"
    "  border: none;"
    "  max-height: 1px;"
    "}");
  drawer_layout->addWidget(divider);

  auto * info_section_label = new QLabel(tr("HELP & INFO"));
  info_section_label->setStyleSheet(
    "QLabel {"
    "  color: #79c0ff;"
    "  font-size: 8pt;"
    "  font-weight: 800;"
    "  letter-spacing: 2px;"
    "  margin-top: 4px;"
    "  margin-bottom: 4px;"
    "}");
  drawer_section_labels_.append(info_section_label);
  drawer_layout->addWidget(info_section_label);

  // Help button
  help_btn_ = new QPushButton(tr("? View Help"));
  help_btn_->setCursor(Qt::PointingHandCursor);
  help_btn_->setStyleSheet(
    "QPushButton {"
    "  background-color: transparent;"
    "  color: #8b949e;"
    "  border: none;"
    "  border-left: 3px solid transparent;"
    "  padding: 10px 12px;"
    "  text-align: left;"
    "  font-size: 11pt;"
    "  font-weight: 500;"
    "}"
    "QPushButton:hover {"
    "  background-color: rgba(121, 192, 255, 0.1);"
    "  color: #79c0ff;"
    "}");
  drawer_layout->addWidget(help_btn_);
  connect(help_btn_, &QPushButton::clicked, this, &MainWindow::setupHelpDialog);

  // Diagnostics button
  diag_btn_ = new QPushButton(tr("📊 Diagnostics"));
  diag_btn_->setCursor(Qt::PointingHandCursor);
  diag_btn_->setStyleSheet(help_btn_->styleSheet());
  drawer_layout->addWidget(diag_btn_);
  connect(diag_btn_, &QPushButton::clicked, this, &MainWindow::launchDiagnosticsWindow);

  updateDrawerActiveState();
}

void MainWindow::setupContentStack()
{
  // Keep one embedded RViz VisualizationManager alive. RViz/Ogre registers default
  // materials globally, so creating a second manager in this process can throw on
  // duplicate material names such as RVIZ/Red.
  auto rviz_sim_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
    "interaction_execution_rviz_panels");
  auto * sim_layout = new QVBoxLayout(ui_->pageSimView);
  sim_layout->setContentsMargins(0, 0, 0, 0);
  rviz_view_ = new RvizSimView(node_, rviz_sim_node, ui_->pageSimView);
  sim_layout->addWidget(rviz_view_);

  auto * task_layout = new QVBoxLayout(ui_->pageTaskView);
  task_layout->setContentsMargins(0, 0, 0, 0);
  hl_digital_twin_view_ = new HlDigitalTwinView(node_, ui_->pageTaskView);
  task_layout->addWidget(hl_digital_twin_view_);

  // Camera View page
  setupCameraPage();

  // Diagnostics page
  setupDiagnosticsWindow();

  // Wordle sidebar
  auto * wordle_layout = new QVBoxLayout(ui_->wordle);
  wordle_layout->setContentsMargins(0, 0, 0, 0);
  wordle_view_ = new WordleView(ui_->wordle);
  wordle_layout->addWidget(wordle_view_);

  ui_->contentStack->setCurrentIndex(0);
}

void MainWindow::setupCameraPage()
{
  auto * layout = new QVBoxLayout(ui_->pageCameraView);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  // Camera mode toggle bar
  auto * toggle_bar = new QWidget();
  toggle_bar->setFixedHeight(32);
  toggle_bar->setStyleSheet(
    "QWidget {"
    "  background-color: #0f151c;"
    "  border-bottom: 1px solid rgba(139, 148, 158, 0.18);"
    "}");
  auto * toggle_layout = new QHBoxLayout(toggle_bar);
  toggle_layout->setContentsMargins(12, 4, 12, 4);
  toggle_layout->setSpacing(6);

  auto * mode_label = new QLabel(tr("Camera:"));
  mode_label->setStyleSheet("QLabel { font-size: 9pt; color: #8b949e; }");
  toggle_layout->addWidget(mode_label);

  cam_raw_btn_ = new QPushButton(tr("Raw"));
  cam_cv_btn_ = new QPushButton(tr("CV"));

  auto style_camera_btn = [](QPushButton * btn, bool active) {
    if (active) {
      btn->setStyleSheet(
        "QPushButton {"
        "  background-color: rgba(31, 111, 235, 0.3);"
        "  color: #79c0ff;"
        "  border: 1px solid rgba(121, 192, 255, 0.3);"
        "  padding: 4px 8px;"
        "  border-radius: 4px;"
        "  font-weight: 600;"
        "  font-size: 9pt;"
        "}");
    } else {
      btn->setStyleSheet(
        "QPushButton {"
        "  background-color: #161f29;"
        "  color: #8b949e;"
        "  border: 1px solid rgba(139, 148, 158, 0.2);"
        "  padding: 4px 8px;"
        "  border-radius: 4px;"
        "  font-size: 9pt;"
        "}"
        "QPushButton:hover {"
        "  background-color: #1f2937;"
        "}");
    }
  };

  style_camera_btn(cam_raw_btn_, true);  // Raw is default
  style_camera_btn(cam_cv_btn_, false);

  toggle_layout->addWidget(cam_raw_btn_);
  toggle_layout->addWidget(cam_cv_btn_);
  toggle_layout->addStretch();

  layout->addWidget(toggle_bar);

  // Camera view
  camera_view_ = new CameraView(node_, ui_->pageCameraView);
  layout->addWidget(camera_view_);

  connect(cam_raw_btn_, &QPushButton::clicked, this, [this]() {
    switchCameraMode(CameraMode::Raw);
  });
  connect(cam_cv_btn_, &QPushButton::clicked, this, [this]() {
    switchCameraMode(CameraMode::ComputerVision);
  });
}

void MainWindow::switchToView(ActiveView view)
{
  active_view_ = view;
  switch (view) {
    case ActiveView::SimView:
      ui_->contentStack->setCurrentIndex(0);
      break;
    case ActiveView::TaskView:
      ui_->contentStack->setCurrentIndex(1);
      break;
    case ActiveView::CameraView:
      ui_->contentStack->setCurrentIndex(2);
      break;
  }
  updateDrawerActiveState();
}

void MainWindow::switchCameraMode(CameraMode mode)
{
  camera_mode_ = mode;
  const QString topic = (mode == CameraMode::Raw)
    ? "/camera/camera/color/image_raw"
    : "/perception/image_annotated";

  camera_view_->setTopic(topic);

  // Update button styling
  auto style_camera_btn = [](QPushButton * btn, bool active) {
    if (active) {
      btn->setStyleSheet(
        "QPushButton {"
        "  background-color: rgba(31, 111, 235, 0.3);"
        "  color: #79c0ff;"
        "  border: 1px solid rgba(121, 192, 255, 0.3);"
        "  padding: 6px 12px;"
        "  border-radius: 4px;"
        "  font-weight: 600;"
        "}");
    } else {
      btn->setStyleSheet(
        "QPushButton {"
        "  background-color: #161f29;"
        "  color: #8b949e;"
        "  border: 1px solid rgba(139, 148, 158, 0.2);"
        "  padding: 6px 12px;"
        "  border-radius: 4px;"
        "}"
        "QPushButton:hover {"
        "  background-color: #1f2937;"
        "}");
    }
  };

  style_camera_btn(cam_raw_btn_, mode == CameraMode::Raw);
  style_camera_btn(cam_cv_btn_, mode == CameraMode::ComputerVision);
}

void MainWindow::toggleDrawer()
{
  drawer_expanded_ = !drawer_expanded_;
  const int target_width = drawer_expanded_ ? 220 : 44;

  auto * anim = new QPropertyAnimation(ui_->drawerWrapper, "maximumWidth", this);
  anim->setDuration(200);
  anim->setEasingCurve(QEasingCurve::InOutQuad);
  anim->setStartValue(ui_->drawerWrapper->maximumWidth());
  anim->setEndValue(target_width);
  anim->start(QAbstractAnimation::DeleteWhenStopped);

  updateDrawerLabelsVisibility();
}

void MainWindow::updateDrawerActiveState()
{
  auto set_button_active = [](QPushButton * btn, bool active) {
    if (active) {
      btn->setStyleSheet(
        "QPushButton {"
        "  background-color: rgba(31, 111, 235, 0.25);"
        "  color: #79c0ff;"
        "  border: none;"
        "  border-left: 3px solid #1f6feb;"
        "  padding: 10px 9px;"
        "  text-align: left;"
        "  font-size: 11pt;"
        "  font-weight: 500;"
        "}"
        "QPushButton:hover {"
        "  background-color: rgba(31, 111, 235, 0.3);"
        "}");
    } else {
      btn->setStyleSheet(
        "QPushButton {"
        "  background-color: transparent;"
        "  color: #8b949e;"
        "  border: none;"
        "  border-left: 3px solid transparent;"
        "  padding: 10px 12px;"
        "  text-align: left;"
        "  font-size: 11pt;"
        "  font-weight: 500;"
        "}"
        "QPushButton:hover {"
        "  background-color: rgba(121, 192, 255, 0.1);"
        "  color: #79c0ff;"
        "}");
    }
  };

  set_button_active(nav_sim_btn_, active_view_ == ActiveView::SimView);
  set_button_active(nav_task_btn_, active_view_ == ActiveView::TaskView);
  set_button_active(nav_camera_btn_, active_view_ == ActiveView::CameraView);
}

void MainWindow::updateDrawerLabelsVisibility()
{
  // Hide all drawer content when collapsed except hamburger button
  for (auto * label : drawer_section_labels_) {
    label->setVisible(drawer_expanded_);
  }

  if (nav_sim_btn_) nav_sim_btn_->setVisible(drawer_expanded_);
  if (nav_task_btn_) nav_task_btn_->setVisible(drawer_expanded_);
  if (nav_camera_btn_) nav_camera_btn_->setVisible(drawer_expanded_);
  if (help_btn_) help_btn_->setVisible(drawer_expanded_);
  if (diag_btn_) diag_btn_->setVisible(drawer_expanded_);
}

void MainWindow::setupVisualDesign()
{
  setStyleSheet(
    "QMainWindow {"
    "  background-color: #0b0f14;"
    "  color: #e6edf3;"
    "}");

  ui_->centralwidget->setStyleSheet(
    "QWidget#centralwidget {"
    "  background-color: #0b0f14;"
    "}");

  ui_->drawerWrapper->setStyleSheet(
    "QWidget#drawerWrapper {"
    "  background-color: #111820;"
    "  border-right: 1px solid rgba(139, 148, 158, 0.18);"
    "}");

  ui_->contentStack->setStyleSheet(
    "QWidget {"
    "  background-color: #0f151c;"
    "}");

  ui_->wordle->setStyleSheet(
    "QWidget#wordle {"
    "  background-color: #0d1218;"
    "  border-left: 1px solid rgba(139, 148, 158, 0.16);"
    "}");

  ui_->voiceControls->setStyleSheet(
    "QWidget#voiceControls {"
    "  background-color: #111820;"
    "  border-top: 1px solid rgba(139, 148, 158, 0.18);"
    "  border-left: 1px solid rgba(139, 148, 158, 0.16);"
    "}"
    "QLabel#voiceLabel {"
    "  color: #79c0ff;"
    "  font-size: 8pt;"
    "  font-weight: 800;"
    "  letter-spacing: 1.5px;"
    "  background: transparent;"
    "}"
    "QFrame#voiceTranscriptFrame {"
    "  background-color: #0b0f14;"
    "  border: 1px solid rgba(121, 192, 255, 0.22);"
    "  border-radius: 8px;"
    "}"
    "QLabel#voiceTranscriptValue {"
    "  color: #f0f6fc;"
    "  font-size: 14pt;"
    "  font-weight: 800;"
    "  background: transparent;"
    "}"
    "QPushButton#voiceRecordButton,"
    "QPushButton#voiceStopButton,"
    "QPushButton#voiceConfirmButton,"
    "QPushButton#voiceRetryButton {"
    "  border-radius: 6px;"
    "  font-size: 9pt;"
    "  font-weight: 800;"
    "  padding: 8px 0px;"
    "}"
    "QPushButton#voiceRecordButton {"
    "  background-color: #1f6feb;"
    "  color: #ffffff;"
    "  border: 1px solid rgba(121, 192, 255, 0.35);"
    "}"
    "QPushButton#voiceRecordButton:hover {"
    "  background-color: #2f81f7;"
    "}"
    "QPushButton#voiceStopButton {"
    "  background-color: #21262d;"
    "  color: #f0f6fc;"
    "  border: 1px solid rgba(240, 246, 252, 0.14);"
    "}"
    "QPushButton#voiceStopButton:hover {"
    "  background-color: #30363d;"
    "}"
    "QPushButton#voiceConfirmButton {"
    "  background-color: #238636;"
    "  color: #ffffff;"
    "  border: 1px solid rgba(86, 211, 100, 0.28);"
    "}"
    "QPushButton#voiceConfirmButton:hover {"
    "  background-color: #2ea043;"
    "}"
    "QPushButton#voiceRetryButton {"
    "  background-color: #21262d;"
    "  color: #c9d1d9;"
    "  border: 1px solid rgba(240, 246, 252, 0.12);"
    "}"
    "QPushButton#voiceRetryButton:hover {"
    "  background-color: #30363d;"
    "}"
    "QPushButton:disabled {"
    "  background-color: #161b22;"
    "  color: #6e7681;"
    "  border-color: rgba(139, 148, 158, 0.12);"
    "}");

  ui_->safetyControls->setStyleSheet(
    "QWidget#safetyControls {"
    "  background-color: #120f10;"
    "  border-top: 1px solid rgba(248, 81, 73, 0.38);"
    "  border-left: 1px solid rgba(139, 148, 158, 0.16);"
    "}"
    "QPushButton {"
    "  border-radius: 6px;"
    "  font-size: 9pt;"
    "  font-weight: 800;"
    "  padding: 8px 0px;"
    "}"
    "QPushButton#pushButton {"
    "  background-color: #f2cc60;"
    "  color: #111820;"
    "  border: 1px solid rgba(242, 204, 96, 0.45);"
    "}"
    "QPushButton#pushButton:hover {"
    "  background-color: #ffd866;"
    "}"
    "QPushButton#pushButton:pressed {"
    "  background-color: #d9a441;"
    "}"
    "QPushButton#pushButton_4 {"
    "  background-color: #da3633;"
    "  color: #ffffff;"
    "  border: 1px solid rgba(248, 81, 73, 0.55);"
    "}"
    "QPushButton#pushButton_4:hover {"
    "  background-color: #f85149;"
    "}"
    "QPushButton#pushButton_4:pressed {"
    "  background-color: #b62324;"
    "}"
    "QPushButton#pushButton_2,"
    "QPushButton#pushButton_3 {"
    "  background-color: #21262d;"
    "  color: #c9d1d9;"
    "  border: 1px solid rgba(240, 246, 252, 0.12);"
    "}"
    "QPushButton#pushButton_2:hover,"
    "QPushButton#pushButton_3:hover {"
    "  background-color: #30363d;"
    "}"
    "QPushButton#pushButton_2:pressed,"
    "QPushButton#pushButton_3:pressed {"
    "  background-color: #161b22;"
    "}"
    "QPushButton:disabled {"
    "  background-color: #161b22;"
    "  color: #6e7681;"
    "  border-color: rgba(139, 148, 158, 0.12);"
    "}");
}

void MainWindow::setupDiagnosticsWindow()
{
  if (diagnostics_mission_value_label_ != nullptr) {
    return;
  }

  // Use pageDiagnostics instead of creating a separate window
  ui_->pageDiagnostics->setStyleSheet(
    "QWidget {"
    "  background-color: #0b0f14;"
    "  color: #e6edf3;"
    "}"
    "QFrame#diagnosticsCard {"
    "  background-color: #111820;"
    "  border: 1px solid rgba(139, 148, 158, 0.18);"
    "  border-radius: 8px;"
    "}"
    "QLabel#diagnosticsSectionLabel {"
    "  color: #79c0ff;"
    "  font-size: 8.5pt;"
    "  font-weight: 800;"
    "  letter-spacing: 1.5px;"
    "}"
    "QLabel#diagnosticsCardTitle {"
    "  color: #8b949e;"
    "  font-size: 8pt;"
    "  font-weight: 800;"
    "  letter-spacing: 1px;"
    "}"
    "QLabel#diagnosticsCardValue {"
    "  color: #f0f6fc;"
    "  font-size: 11pt;"
    "  font-weight: 800;"
    "}"
    "QPlainTextEdit#diagnosticsTextPane {"
    "  background-color: #0b0f14;"
    "  color: #c9d1d9;"
    "  selection-background-color: #1f6feb;"
    "  border: 1px solid rgba(139, 148, 158, 0.20);"
    "  border-radius: 8px;"
    "  padding: 8px;"
    "}");

  auto * root_layout = new QVBoxLayout(ui_->pageDiagnostics);
  root_layout->setContentsMargins(18, 18, 18, 18);
  root_layout->setSpacing(14);

  // ── MISSION SECTION ──────────────────────
  auto * mission_header = new QLabel(tr("CURRENT MISSION"));
  mission_header->setObjectName("diagnosticsSectionLabel");
  root_layout->addWidget(mission_header);

  diag_mission_title_ = new QLabel(tr("Wordle Game Pick and Place"));
  diag_mission_title_->setStyleSheet(
    "QLabel {"
    "  color: #f0f6fc;"
    "  font-size: 13pt;"
    "  font-weight: 800;"
    "}");
  diag_mission_title_->setWordWrap(true);
  root_layout->addWidget(diag_mission_title_);

  diag_mission_summary_ = new QLabel(tr("Awaiting mission progress from coordinator."));
  diag_mission_summary_->setStyleSheet(
    "QLabel {"
    "  color: #c9d1d9;"
    "  font-size: 9pt;"
    "}");
  diag_mission_summary_->setWordWrap(true);
  root_layout->addWidget(diag_mission_summary_);

  diag_mission_scroll_ = new QScrollArea();
  diag_mission_scroll_->setWidgetResizable(true);
  diag_mission_scroll_->setFrameShape(QFrame::NoFrame);
  diag_mission_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  diag_mission_scroll_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  diag_mission_scroll_->setStyleSheet(
    "QScrollArea { background: transparent; }"
    "QScrollArea > QWidget > QWidget { background: transparent; }"
    "QScrollBar:vertical { background: transparent; width: 8px; }"
    "QScrollBar::handle:vertical { background: rgba(139, 148, 158, 0.40); border-radius: 4px; }"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }");

  diag_mission_steps_content_ = new QWidget(diag_mission_scroll_);
  diag_mission_steps_layout_ = new QVBoxLayout(diag_mission_steps_content_);
  diag_mission_steps_layout_->setContentsMargins(0, 0, 0, 0);
  diag_mission_steps_layout_->setSpacing(10);
  diag_mission_steps_layout_->addStretch();
  diag_mission_scroll_->setWidget(diag_mission_steps_content_);
  diag_mission_scroll_->setMinimumHeight(100);
  diag_mission_scroll_->setMaximumHeight(150);
  root_layout->addWidget(diag_mission_scroll_);

  // Divider
  auto * divider = new QFrame();
  divider->setFrameShape(QFrame::HLine);
  divider->setStyleSheet("QFrame { background-color: rgba(139, 148, 158, 0.18); border: none; }");
  root_layout->addWidget(divider);

  // ──────────────────────────────────────────

  auto * summary_label = new QLabel(tr("SYSTEM SUMMARY"));
  summary_label->setObjectName("diagnosticsSectionLabel");
  root_layout->addWidget(summary_label);

  auto * cards_layout = new QGridLayout();
  cards_layout->setHorizontalSpacing(12);
  cards_layout->setVerticalSpacing(12);

  const auto create_card = [this](const QString & title, QLabel ** value_label) {
    auto * card = new QFrame();
    card->setObjectName("diagnosticsCard");

    auto * layout = new QVBoxLayout(card);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(6);

    auto * title_label = new QLabel(title);
    title_label->setObjectName("diagnosticsCardTitle");
    layout->addWidget(title_label);

    auto * value = new QLabel(tr("Waiting for data"));
    value->setObjectName("diagnosticsCardValue");
    value->setWordWrap(true);
    value->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    layout->addWidget(value);

    *value_label = value;
    return card;
  };

  cards_layout->addWidget(create_card(tr("Mission"), &diagnostics_mission_value_label_), 0, 0);
  cards_layout->addWidget(create_card(tr("Safety"), &diagnostics_safety_value_label_), 0, 1);
  cards_layout->addWidget(create_card(tr("Perception"), &diagnostics_perception_value_label_), 1, 0);
  cards_layout->addWidget(create_card(tr("Wordle"), &diagnostics_wordle_value_label_), 1, 1);
  root_layout->addLayout(cards_layout);

  auto * lower_layout = new QGridLayout();
  lower_layout->setHorizontalSpacing(12);
  lower_layout->setVerticalSpacing(12);

  auto * events_label = new QLabel(tr("RECENT EVENTS"));
  events_label->setObjectName("diagnosticsSectionLabel");
  lower_layout->addWidget(events_label, 0, 0);

  diagnostics_event_log_ = new QPlainTextEdit();
  diagnostics_event_log_->setObjectName("diagnosticsTextPane");
  diagnostics_event_log_->setReadOnly(true);
  diagnostics_event_log_->setMinimumHeight(180);
  diagnostics_event_log_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  lower_layout->addWidget(diagnostics_event_log_, 1, 0);

  auto * mission_json_label = new QLabel(tr("MISSION PROGRESS JSON"));
  mission_json_label->setObjectName("diagnosticsSectionLabel");
  lower_layout->addWidget(mission_json_label, 0, 1);

  diagnostics_mission_json_view_ = new QPlainTextEdit();
  diagnostics_mission_json_view_->setObjectName("diagnosticsTextPane");
  diagnostics_mission_json_view_->setReadOnly(true);
  diagnostics_mission_json_view_->setMinimumHeight(180);
  diagnostics_mission_json_view_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  lower_layout->addWidget(diagnostics_mission_json_view_, 1, 1);

  auto * game_json_label = new QLabel(tr("WORDLE DIAGNOSTICS JSON"));
  game_json_label->setObjectName("diagnosticsSectionLabel");
  lower_layout->addWidget(game_json_label, 2, 0, 1, 2);

  diagnostics_game_json_view_ = new QPlainTextEdit();
  diagnostics_game_json_view_->setObjectName("diagnosticsTextPane");
  diagnostics_game_json_view_->setReadOnly(true);
  diagnostics_game_json_view_->setMinimumHeight(180);
  diagnostics_game_json_view_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  lower_layout->addWidget(diagnostics_game_json_view_, 3, 0, 1, 2);

  root_layout->addLayout(lower_layout);
  root_layout->addStretch();

  // Mission progress subscription
  mission_progress_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kMissionProgressTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }
      renderMissionProgress(QString::fromStdString(msg->data));
    });

  // Initialize mission progress
  renderMissionProgress(QStringLiteral(
    "{\"title\":\"Wordle Game Pick and Place\",\"summary\":\"Awaiting mission progress from coordinator.\",\"steps\":[]}"));

  refreshDiagnosticsPanel();
}

void MainWindow::setupVoiceControls()
{
  ui_->voiceRecordButton->setIcon(makeRecordIcon());
  ui_->voiceRecordButton->setIconSize(QSize(18, 18));
  ui_->voiceStopButton->hide();
  ui_->voiceConfirmButton->setIcon(makeConfirmIcon());
  ui_->voiceConfirmButton->setIconSize(QSize(18, 18));
  ui_->voiceRetryButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  ui_->voiceRetryButton->setIconSize(QSize(18, 18));

  connect(ui_->voiceRecordButton, &QPushButton::clicked, this, [this]() {
    if (voice_recording_) {
      stopVoiceRecording();
      return;
    }

    beginVoiceRecording(true);
  });

  connect(ui_->voiceConfirmButton, &QPushButton::clicked, this, [this]() {
    if (pending_voice_guess_.isEmpty()) {
      return;
    }

    const QString confirmed_guess = pending_voice_guess_.toUpper();
    publishGamificationPlayerGuess(confirmed_guess);
    setVoicePreviewText(tr("Submitted: %1").arg(confirmed_guess));
    pending_voice_guess_.clear();
    voice_result_pending_ = false;
    voice_retry_rejections_ = 0;
    updateVoiceControlsState();
  });

  connect(ui_->voiceRetryButton, &QPushButton::clicked, this, [this]() {
    if (voice_recording_ || voice_transcribing_) {
      sendVoiceHelperCommand(QStringLiteral("cancel_recording"));
    }

    if (voice_retry_rejections_ >= 1) {
      promptManualVoiceOverride();
      return;
    }

    voice_retry_rejections_ += 1;
    beginVoiceRecording(false);
  });

  resetVoicePreview(tr("Select game mode"));
  updateVoiceControlsState();
}

void MainWindow::beginVoiceRecording(bool reset_retry_sequence)
{
  if (
    current_wordle_mode_ != WordleGameMode::ModeB || !voice_helper_available_ ||
    voice_helper_process_ == nullptr || voice_transcribing_)
  {
    return;
  }

  if (reset_retry_sequence) {
    voice_retry_rejections_ = 0;
  }

  wordle_view_->clearPreviewGuess();
  pending_voice_guess_.clear();
  voice_result_pending_ = false;
  voice_recording_ = true;
  voice_transcribing_ = false;
  setVoicePreviewText(tr("Listening..."));
  sendVoiceHelperCommand(QStringLiteral("start_recording"));
  updateVoiceControlsState();
}

void MainWindow::stopVoiceRecording()
{
  if (!voice_recording_ || voice_helper_process_ == nullptr) {
    return;
  }

  voice_recording_ = false;
  voice_transcribing_ = true;
  setVoicePreviewText(tr("Transcribing..."));
  sendVoiceHelperCommand(QStringLiteral("stop_recording"));
  updateVoiceControlsState();
}

void MainWindow::promptManualVoiceOverride()
{
  const auto choice = QMessageBox::question(
    this,
    tr("Manual Override"),
    tr("Voice input still does not match. Manually type the word?"),
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::Yes);
  if (choice != QMessageBox::Yes) {
    beginVoiceRecording(false);
    return;
  }

  QString typed_word = pending_voice_guess_;
  while (true) {
    bool accepted = false;
    typed_word = QInputDialog::getText(
      this,
      tr("Manual Word Override"),
      tr("Word"),
      QLineEdit::Normal,
      typed_word,
      &accepted).trimmed().toUpper();

    if (!accepted) {
      updateVoiceControlsState();
      return;
    }

    if (isValidWordleWord(typed_word)) {
      break;
    }

    QMessageBox::warning(
      this,
      tr("Invalid Word"),
      tr("Enter a valid five-letter Wordle word from the dictionary."));
  }

  pending_voice_guess_ = typed_word;
  voice_recording_ = false;
  voice_transcribing_ = false;
  voice_result_pending_ = true;
  wordle_view_->previewGuess(pending_voice_guess_);
  setVoicePreviewText(tr("Override: %1").arg(pending_voice_guess_));
  updateVoiceControlsState();
}

bool MainWindow::isValidWordleWord(const QString & word) const
{
  const QString cleaned = word.trimmed().toLower();
  if (cleaned.size() != 5 ||
    !std::all_of(cleaned.cbegin(), cleaned.cend(), [](const QChar ch) { return ch.isLetter(); }))
  {
    return false;
  }

  return wordle_dictionary_.contains(cleaned);
}

void MainWindow::setupVoiceHelper()
{
  QString package_share_dir;
  try {
    package_share_dir = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("interaction_execution"));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to locate interaction_execution share directory for voice helper lookup: %s",
      ex.what());
    setVoicePreviewText(tr("Voice helper unavailable"));
    updateVoiceControlsState();
    return;
  }

  const QString helper_path = package_share_dir + "/voice_control/gui_bridge.py";
  if (!QFileInfo::exists(helper_path)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Voice helper script not found at %s",
      helper_path.toStdString().c_str());
    setVoicePreviewText(tr("Voice helper unavailable"));
    updateVoiceControlsState();
    return;
  }

  voice_helper_process_ = new QProcess(this);
  voice_helper_process_->setProgram(QStringLiteral("/bin/python3"));
  voice_helper_process_->setArguments({
    helper_path,
    QStringLiteral("--dictionary"),
    wordle_dictionary_path_
  });

  connect(
    voice_helper_process_, &QProcess::readyReadStandardOutput, this,
    &MainWindow::handleVoiceHelperStdout);
  connect(
    voice_helper_process_, &QProcess::readyReadStandardError, this,
    &MainWindow::handleVoiceHelperStderr);
  connect(
    voice_helper_process_,
    qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
    this,
    [this](int exit_code, QProcess::ExitStatus exit_status) {
      voice_recording_ = false;
      voice_transcribing_ = false;
      voice_result_pending_ = false;
      voice_helper_available_ = false;
      updateVoiceControlsState();

      if (exit_status == QProcess::CrashExit) {
        setVoicePreviewText(tr("Voice helper crashed"));
        RCLCPP_ERROR(node_->get_logger(), "Voice helper crashed.");
        return;
      }

      if (exit_code != 0) {
        setVoicePreviewText(tr("Voice helper stopped"));
        RCLCPP_ERROR(
          node_->get_logger(),
          "Voice helper exited with code %d.",
          exit_code);
      }
    });
  connect(
    voice_helper_process_,
    &QProcess::errorOccurred,
    this,
    [this](QProcess::ProcessError error) {
      voice_recording_ = false;
      voice_transcribing_ = false;
      voice_result_pending_ = false;
      voice_helper_available_ = false;
      updateVoiceControlsState();
      setVoicePreviewText(tr("Voice helper unavailable"));
      RCLCPP_ERROR(
        node_->get_logger(),
        "Voice helper process error: %d",
        static_cast<int>(error));
    });

  voice_helper_process_->start();
  if (!voice_helper_process_->waitForStarted(1500)) {
    setVoicePreviewText(tr("Voice helper unavailable"));
    updateVoiceControlsState();
    RCLCPP_ERROR(node_->get_logger(), "Failed to start voice helper.");
    return;
  }

  voice_helper_available_ = true;
  resetVoicePreview(
    current_wordle_mode_ == WordleGameMode::ModeB ?
    tr("Awaiting input...") :
    (current_wordle_mode_ == WordleGameMode::ModeA ?
    tr("Mode B voice input only") : tr("Select game mode")));
  updateVoiceControlsState();
}

void MainWindow::handleVoiceHelperStdout()
{
  if (voice_helper_process_ == nullptr) {
    return;
  }

  voice_helper_stdout_buffer_ += QString::fromUtf8(voice_helper_process_->readAllStandardOutput());

  int newline_index = voice_helper_stdout_buffer_.indexOf('\n');
  while (newline_index >= 0) {
    const QString line = voice_helper_stdout_buffer_.left(newline_index).trimmed();
    voice_helper_stdout_buffer_.remove(0, newline_index + 1);
    if (!line.isEmpty()) {
      handleVoiceHelperMessage(line);
    }
    newline_index = voice_helper_stdout_buffer_.indexOf('\n');
  }
}

void MainWindow::handleVoiceHelperStderr()
{
  if (voice_helper_process_ == nullptr) {
    return;
  }

  const QString stderr_output = QString::fromUtf8(voice_helper_process_->readAllStandardError());
  for (const QString & line : stderr_output.split('\n', Qt::SkipEmptyParts)) {
    RCLCPP_WARN(node_->get_logger(), "Voice helper: %s", line.trimmed().toStdString().c_str());
  }
}

void MainWindow::handleVoiceHelperMessage(const QString & payload)
{
  const QJsonDocument document = QJsonDocument::fromJson(payload.toUtf8());
  if (!document.isObject()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring non-JSON voice helper payload: %s",
      payload.toStdString().c_str());
    return;
  }

  const QJsonObject object = document.object();
  const QString event = object.value("event").toString();

  if (event == "ready") {
    voice_helper_available_ = true;
    resetVoicePreview(
      current_wordle_mode_ == WordleGameMode::ModeB ?
      tr("Awaiting input...") :
      (current_wordle_mode_ == WordleGameMode::ModeA ?
      tr("Mode B voice input only") : tr("Select game mode")));
    updateVoiceControlsState();
    return;
  }

  if (event == "recording_started") {
    voice_recording_ = true;
    voice_transcribing_ = false;
    setVoicePreviewText(tr("Listening..."));
    updateVoiceControlsState();
    return;
  }

  if (event == "recording_cancelled") {
    voice_recording_ = false;
    voice_transcribing_ = false;
    voice_result_pending_ = false;
    pending_voice_guess_.clear();
    wordle_view_->clearPreviewGuess();
    resetVoicePreview(
      current_wordle_mode_ == WordleGameMode::ModeB ?
      tr("Awaiting input...") :
      (current_wordle_mode_ == WordleGameMode::ModeA ?
      tr("Mode B voice input only") : tr("Select game mode")));
    updateVoiceControlsState();
    return;
  }

  if (event == "recording_result") {
    voice_recording_ = false;
    voice_transcribing_ = false;
    voice_result_pending_ = true;

    const QString transcript = object.value("transcript").toString().trimmed();
    const QString guessed_word = object.value("guess").toString().trimmed().toUpper();

    pending_voice_guess_.clear();
    wordle_view_->clearPreviewGuess();

    if (!guessed_word.isEmpty()) {
      pending_voice_guess_ = guessed_word;
      wordle_view_->previewGuess(guessed_word);
      setVoicePreviewText(tr("Heard: %1").arg(guessed_word));
    } else if (!transcript.isEmpty()) {
      setVoicePreviewText(tr("Heard: %1").arg(transcript));
    } else {
      setVoicePreviewText(tr("No speech recognised"));
    }

    updateVoiceControlsState();
    return;
  }

  if (event == "error") {
    voice_recording_ = false;
    voice_transcribing_ = false;
    voice_result_pending_ = true;
    pending_voice_guess_.clear();
    wordle_view_->clearPreviewGuess();

    const QString message = object.value("message").toString().trimmed();
    setVoicePreviewText(message.isEmpty() ? tr("Voice input failed") : message);
    updateVoiceControlsState();
  }
}

void MainWindow::sendVoiceHelperCommand(const QString & command)
{
  if (voice_helper_process_ == nullptr || voice_helper_process_->state() != QProcess::Running) {
    return;
  }

  QJsonObject payload;
  payload.insert(QStringLiteral("command"), command);
  voice_helper_process_->write(QJsonDocument(payload).toJson(QJsonDocument::Compact));
  voice_helper_process_->write("\n");
}

void MainWindow::sendVoiceHelperShutdown()
{
  if (voice_helper_process_ == nullptr || voice_helper_process_->state() != QProcess::Running) {
    return;
  }

  sendVoiceHelperCommand(QStringLiteral("shutdown"));
  voice_helper_process_->waitForBytesWritten(250);
}

void MainWindow::setVoicePreviewText(const QString & text)
{
  voice_preview_text_ = text;
  ui_->voiceTranscriptValue->setText(voice_preview_text_);
}

void MainWindow::resetVoicePreview(const QString & text)
{
  setVoicePreviewText(text);
}

void MainWindow::setupGamificationBridge()
{
  gamification_feedback_pub_ =
    node_->create_publisher<std_msgs::msg::String>(kGamificationFeedbackTopic, 10);
  gamification_mode_pub_ =
    node_->create_publisher<std_msgs::msg::String>(kGamificationModeTopic, 10);
  gamification_secret_word_pub_ =
    node_->create_publisher<std_msgs::msg::String>(kGamificationSecretWordTopic, 10);
  gamification_player_guess_pub_ =
    node_->create_publisher<std_msgs::msg::String>(kGamificationPlayerGuessTopic, 10);

  connect(wordle_view_, &WordleView::feedbackSubmitted, this, [this](const QString & feedback) {
    publishGamificationFeedback(feedback);
  });
  connect(wordle_view_, &WordleView::modeSelected, this, [this](const QString & mode) {
    publishGamificationMode(mode);
  });
  connect(
    wordle_view_, &WordleView::secretWordSubmitted, this,
    [this](const QString & word) {
      publishGamificationSecretWord(word);
    });
  connect(
    wordle_view_, &WordleView::playerGuessSubmitted, this,
    [this](const QString & guess) {
      publishGamificationPlayerGuess(guess);
    });
  connect(wordle_view_, &WordleView::resetRequested, this, [this]() {
    resetGamificationGame();
  });

  gamification_guess_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kGamificationGuessTopic,
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      const QString guess = QString::fromStdString(msg->data).trimmed().toUpper();
      if (guess.isEmpty()) {
        return;
      }

      wordle_view_->setActiveGuess(guess);
      current_wordle_guess_ = guess;
      appendDiagnosticsEvent(tr("Robot guess updated to %1").arg(guess));
      refreshDiagnosticsPanel();
    });

  gamification_diagnostics_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kDiagnosticsTopic,
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      const QString payload = QString::fromStdString(msg->data);
      last_gamification_diagnostics_payload_ = payload;
      wordle_view_->setDiagnosticsJson(payload);

      const QJsonDocument document = QJsonDocument::fromJson(payload.toUtf8());
      if (!document.isObject()) {
        diagnostics_game_json_view_->setPlainText(payload);
        return;
      }

      const QJsonObject object = document.object();
      const QString status = object.value("status").toString("UNKNOWN");
      const QString mode = object.value("mode").toString("A").toUpper();
      const bool mode_locked = object.value("mode_locked").toBool(false);
      const int attempt = object.value("attempt").toInt(0);
      const QString guess = object.value("current_guess").toString();
      if (!mode_locked) {
        current_wordle_mode_ = WordleGameMode::Unset;
      } else {
        current_wordle_mode_ = mode == "B" ? WordleGameMode::ModeB : WordleGameMode::ModeA;
      }
      current_wordle_status_ = status;
      current_wordle_attempt_ = attempt;
      current_wordle_guess_ = guess;
      current_candidates_left_ = object.value("candidates_left").toInt(0);
      updateVoiceControlsState();

      QString summary = mode_locked ?
        tr("Mode %1 | Status: %2").arg(
          mode == "B" ? QStringLiteral("B") : QStringLiteral("A"), status) :
        tr("Mode: Select | Status: %1").arg(status);
      if (attempt > 0) {
        summary += tr(" | Attempt %1").arg(attempt);
      }
      if (!guess.isEmpty()) {
        summary += tr(" | Guess %1").arg(guess);
      }

      appendDiagnosticsEvent(
        tr("Wordle diagnostics: %1 | Attempt %2").arg(status).arg(attempt > 0 ? attempt : 0));
      refreshDiagnosticsPanel();
    });
}

void MainWindow::updateVoiceControlsState()
{
  const bool mode_b = current_wordle_mode_ == WordleGameMode::ModeB;
  const bool terminal_game =
    current_wordle_status_ == QStringLiteral("SOLVED") ||
    current_wordle_status_ == QStringLiteral("GAME_OVER");
  const bool busy = voice_recording_ || voice_transcribing_;
  const bool show_result_actions = mode_b && !terminal_game && voice_result_pending_ && !busy;

  ui_->voiceStopButton->setVisible(false);
  ui_->voiceRecordButton->setVisible(!show_result_actions || busy);
  ui_->voiceConfirmButton->setVisible(show_result_actions);
  ui_->voiceRetryButton->setVisible(show_result_actions);

  ui_->voiceRecordButton->setEnabled(
    voice_helper_available_ && mode_b && !terminal_game && !voice_transcribing_);
  ui_->voiceConfirmButton->setEnabled(show_result_actions && !pending_voice_guess_.isEmpty());
  ui_->voiceRetryButton->setEnabled(show_result_actions);

  if (voice_recording_) {
    ui_->voiceRecordButton->setText(tr("Stop"));
    ui_->voiceRecordButton->setIcon(makeStopIcon());
  } else if (voice_transcribing_) {
    ui_->voiceRecordButton->setText(tr("Transcribing"));
    ui_->voiceRecordButton->setIcon(makeStopIcon());
  } else {
    ui_->voiceRecordButton->setText(tr("Record"));
    ui_->voiceRecordButton->setIcon(makeRecordIcon());
  }
}

void MainWindow::setupSafetyControls()
{
  mission_state_pub_ = node_->create_publisher<std_msgs::msg::String>(kPerceptionStateTopic, 10);
  mission_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>(kMissionCommandTopic, 10);
  start_mission_pub_ = node_->create_publisher<std_msgs::msg::Bool>(kStartMissionTopic, 10);
  resume_mission_pub_ = node_->create_publisher<std_msgs::msg::Bool>(kResumeMissionTopic, 10);
  stop_mission_pub_ = node_->create_publisher<std_msgs::msg::Bool>(kStopMissionTopic, 10);
  scan_and_sweep_pub_ = node_->create_publisher<std_msgs::msg::Bool>(kScanAndSweepTopic, 10);
  abort_mission_pub_ = node_->create_publisher<std_msgs::msg::Bool>(kAbortMissionTopic, 10);
  ui_->pushButton->setText(tr("START"));
  ui_->pushButton_2->setText(tr("SCAN GAME BOARD"));
  perception_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kPerceptionStateTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      const QString next_state = QString::fromStdString(msg->data).trimmed().toUpper();
      if (next_state != current_perception_state_) {
        current_perception_state_ = next_state;
        appendDiagnosticsEvent(tr("Perception state changed to %1").arg(next_state));
        refreshDiagnosticsPanel();
      }
    });
  perception_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kPerceptionStatusTopic,
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      current_perception_status_ = QString::fromStdString(msg->data).trimmed();
      refreshDiagnosticsPanel();
    });
  perception_detections_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kPerceptionDetectionsTopic,
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      int detection_count = 0;
      const QString payload = QString::fromStdString(msg->data);
      const QJsonDocument document = QJsonDocument::fromJson(payload.toUtf8());
      if (document.isObject()) {
        const QJsonArray blocks = document.object().value("blocks").toArray();
        detection_count = blocks.size();
      } else {
        detection_count = payload.count("\"letter\"");
      }

      current_detection_count_ = detection_count;
      refreshDiagnosticsPanel();
    });
  mission_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kCoordinatorMissionStateTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      applyCoordinatorMissionState(QString::fromStdString(msg->data), true);
      updateSafetyControlsState();
      refreshDiagnosticsPanel();
    });
  robot_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kRobotStateTopic,
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      const QString robot_state = QString::fromStdString(msg->data).trimmed().toUpper();
      if (robot_state.isEmpty()) {
        return;
      }
      if (robot_state == current_robot_state_ && !(robot_state == "IDLE" && scan_game_board_active_)) {
        return;
      }

      current_robot_state_ = robot_state;

      if (robot_state == "IDLE" && scan_game_board_active_) {
        scan_game_board_active_ = false;
        publishMissionState("IDLE");
        applyCoordinatorMissionState("IDLE", true);
        appendDiagnosticsEvent(tr("Scan game board complete"));
      }

      appendDiagnosticsEvent(tr("Robot state changed to %1").arg(current_robot_state_));
      updateSafetyControlsState();
      refreshDiagnosticsPanel();
    });
  mission_complete_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    kMissionCompleteTopic,
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg == nullptr || !msg->data) {
        return;
      }

      scan_game_board_active_ = false;
      current_robot_state_ = "IDLE";
      applyCoordinatorMissionState("IDLE", true);
      appendDiagnosticsEvent(tr("Mission complete"));
      updateSafetyControlsState();
      refreshDiagnosticsPanel();
    });

  human_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    kHumanDetectedTopic,
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      const bool was_human_detected = human_detected_;
      human_detected_ = msg->data;

      if (!human_detected_) {
        updateSafetyControlsState();
        return;
      }

      if (was_human_detected) {
        return;
      }

      RCLCPP_WARN(
        node_->get_logger(),
        "Human detected on %s. Publishing IDLE + STOP safety commands.",
        kHumanDetectedTopic);
      publishMissionState("IDLE");
      publishMissionCommand("STOP");
      scan_game_board_active_ = false;
      coordinator_mission_state_ = "STOPPED";
      safety_mode_ = SafetyControlMode::Stopped;
      appendDiagnosticsEvent(tr("Safety stop triggered by human detection"));
      updateSafetyControlsState();
      refreshDiagnosticsPanel();
    });

  connect(ui_->pushButton, &QPushButton::clicked, this, [this]() {
    if (safety_mode_ != SafetyControlMode::Idle && safety_mode_ != SafetyControlMode::Stopped) {
      return;
    }

    if (human_detected_) {
      RCLCPP_WARN(node_->get_logger(), "START/RESUME blocked because a human is currently detected.");
      updateSafetyControlsState();
      return;
    }

    const bool resume_requested =
      current_robot_state_ == "RUNNING" || current_robot_state_ == "STOPPED";
    publishMissionCommand(resume_requested ? "RESUME" : "START");
    if (resume_requested) {
      publishMissionSignal(resume_mission_pub_, kResumeMissionTopic);
      applyCoordinatorMissionState("MOVING", false);
    } else {
      applyCoordinatorMissionState("SCANNING", false);
    }
    appendDiagnosticsEvent(
      resume_requested ? tr("Operator command: RESUME") : tr("Operator command: START"));
    updateSafetyControlsState();
    refreshDiagnosticsPanel();
  });

  connect(ui_->pushButton_4, &QPushButton::clicked, this, [this]() {
    // Context-specific button: STOP when Active/Homing, ABORT when Stopped
    if (coordinator_mission_state_ == "STOPPED" && safety_mode_ == SafetyControlMode::Stopped) {
      publishMissionSignal(abort_mission_pub_, kAbortMissionTopic);
      appendDiagnosticsEvent(tr("Operator command: ABORT"));
      RCLCPP_INFO(node_->get_logger(), "Abort mission signal sent.");
      return;
    }

    if (safety_mode_ != SafetyControlMode::Active && safety_mode_ != SafetyControlMode::Homing) {
      return;
    }

    publishMissionState("IDLE");
    publishMissionCommand("STOP");
    publishMissionSignal(stop_mission_pub_, kStopMissionTopic);
    scan_game_board_active_ = false;
    coordinator_mission_state_ = "STOPPED";
    safety_mode_ = SafetyControlMode::Stopped;
    appendDiagnosticsEvent(tr("Operator command: STOP"));
    updateSafetyControlsState();
    refreshDiagnosticsPanel();
  });

  connect(ui_->pushButton_2, &QPushButton::clicked, this, [this]() {
    if (safety_mode_ != SafetyControlMode::Idle && safety_mode_ != SafetyControlMode::Stopped) {
      return;
    }

    if (human_detected_) {
      RCLCPP_WARN(node_->get_logger(), "Scan game board blocked because a human is currently detected.");
      updateSafetyControlsState();
      return;
    }

    scan_game_board_active_ = true;
    publishMissionState("SCANNING");
    publishScanAndSweep();
    coordinator_mission_state_ = "SCANNING";
    safety_mode_ = SafetyControlMode::Active;
    appendDiagnosticsEvent(tr("Operator command: SCAN GAME BOARD"));
    updateSafetyControlsState();
    refreshDiagnosticsPanel();
  });

  connect(ui_->pushButton_3, &QPushButton::clicked, this, [this]() {
    if (safety_mode_ != SafetyControlMode::Stopped) {
      return;
    }

    if (human_detected_) {
      RCLCPP_WARN(node_->get_logger(), "HOME blocked because a human is currently detected.");
      updateSafetyControlsState();
      return;
    }

    publishMissionCommand("HOME");
    coordinator_mission_state_ = "HOMING";
    safety_mode_ = SafetyControlMode::Homing;
    appendDiagnosticsEvent(tr("Operator command: HOME"));
    updateSafetyControlsState();
    refreshDiagnosticsPanel();
  });

  updateSafetyControlsState();
  refreshDiagnosticsPanel();
}

void MainWindow::updateSafetyControlsState()
{
  const bool can_start_or_resume =
    !human_detected_ &&
    (safety_mode_ == SafetyControlMode::Idle || safety_mode_ == SafetyControlMode::Stopped);
  const bool can_stop =
    safety_mode_ == SafetyControlMode::Active || safety_mode_ == SafetyControlMode::Homing;
  const bool can_abort =
    !human_detected_ && safety_mode_ == SafetyControlMode::Stopped && coordinator_mission_state_ == "STOPPED";
  const bool can_scan_game_board =
    !human_detected_ &&
    (safety_mode_ == SafetyControlMode::Idle || safety_mode_ == SafetyControlMode::Stopped);
  const bool can_home = !human_detected_ && safety_mode_ == SafetyControlMode::Stopped;
  const bool show_resume =
    current_robot_state_ == "RUNNING" || current_robot_state_ == "STOPPED";

  ui_->pushButton->setText(show_resume ? tr("RESUME") : tr("START"));
  ui_->pushButton->setEnabled(can_start_or_resume);
  ui_->pushButton_4->setText((can_abort) ? tr("ABORT") : tr("STOP"));
  ui_->pushButton_4->setEnabled(can_stop || can_abort);
  ui_->pushButton_2->setText(tr("SCAN GAME BOARD"));
  ui_->pushButton_2->setEnabled(can_scan_game_board);
  ui_->pushButton_3->setEnabled(can_home);

  if (human_detected_) {
    updateSafetyBanner("SAFETY CONTROLS | HUMAN DETECTED", "#f85149");
    return;
  }

  switch (safety_mode_) {
    case SafetyControlMode::Idle:
      updateSafetyBanner("SAFETY CONTROLS | IDLE", "#f2cc60");
      return;
    case SafetyControlMode::Stopped:
      if (coordinator_mission_state_ == "SAFETY_STOPPED") {
        updateSafetyBanner("SAFETY CONTROLS | SAFETY STOPPED", "#f85149");
      } else if (coordinator_mission_state_ == "PERCEPTION_FAILED") {
        updateSafetyBanner("SAFETY CONTROLS | PERCEPTION FAILED", "#f85149");
      } else if (coordinator_mission_state_ == "MOTION_FAILED") {
        updateSafetyBanner("SAFETY CONTROLS | MOTION FAILED", "#f85149");
      } else if (coordinator_mission_state_ == "ERROR") {
        updateSafetyBanner("SAFETY CONTROLS | ERROR", "#f85149");
      } else {
        updateSafetyBanner("SAFETY CONTROLS | STOPPED", "#f85149");
      }
      return;
    case SafetyControlMode::Homing:
      updateSafetyBanner("SAFETY CONTROLS | RETURN HOME", "#f2cc60");
      return;
    case SafetyControlMode::Active:
      if (coordinator_mission_state_ == "RECOVERING") {
        updateSafetyBanner("SAFETY CONTROLS | RECOVERING", "#f2cc60");
      } else if (current_robot_state_ == "RUNNING") {
        const bool perception_scanning =
          current_perception_status_.trimmed().toUpper() == QStringLiteral("SCANNING");
        updateSafetyBanner(
          perception_scanning ?
            "SAFETY CONTROLS | SCANNING" :
            "SAFETY CONTROLS | ROBOT RUNNING",
          "#56d364");
      } else if (coordinator_mission_state_ == "MOVING") {
        updateSafetyBanner("SAFETY CONTROLS | ACTIVE", "#56d364");
      } else if (coordinator_mission_state_ == "READY_TO_MOVE") {
        updateSafetyBanner("SAFETY CONTROLS | READY", "#56d364");
      } else if (coordinator_mission_state_ == "SCANNING") {
        updateSafetyBanner("SAFETY CONTROLS | SCANNING", "#56d364");
      } else {
        updateSafetyBanner("SAFETY CONTROLS | ACTIVE", "#56d364");
      }
      return;
  }
}

void MainWindow::reserveSidebarWidth()
{
  const QString widest_safety_text = tr("SAFETY CONTROLS | ROBOT RUNNING");
  const QString current_safety_text = ui_->safetyLabel->text();

  ui_->wordle->ensurePolished();
  ui_->voiceControls->ensurePolished();
  ui_->safetyControls->ensurePolished();

  ui_->safetyLabel->setText(widest_safety_text);
  ui_->safetyLabel->ensurePolished();

  const int sidebar_width = std::max({
    ui_->wordle->minimumSizeHint().width(),
    ui_->voiceControls->minimumSizeHint().width(),
    ui_->safetyControls->minimumSizeHint().width()});

  ui_->safetyLabel->setText(current_safety_text);

  ui_->wordle->setMinimumWidth(std::max(ui_->wordle->minimumWidth(), sidebar_width));
  ui_->voiceControls->setMinimumWidth(std::max(ui_->voiceControls->minimumWidth(), sidebar_width));
  ui_->safetyControls->setMinimumWidth(std::max(ui_->safetyControls->minimumWidth(), sidebar_width));
}

void MainWindow::publishMissionState(const std::string & state)
{
  std_msgs::msg::String msg;
  msg.data = state;
  mission_state_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published %s='%s'.", kPerceptionStateTopic, state.c_str());
}

void MainWindow::publishMissionCommand(const std::string & command)
{
  std_msgs::msg::String msg;
  msg.data = command;
  mission_cmd_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published %s='%s'.", kMissionCommandTopic, command.c_str());
}

void MainWindow::publishMissionSignal(
  const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher,
  const char * topic_name)
{
  if (publisher == nullptr) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  publisher->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published %s=true.", topic_name);
}

void MainWindow::publishScanAndSweep()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  scan_and_sweep_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published %s=true.", kScanAndSweepTopic);
}

void MainWindow::publishGamificationFeedback(const QString & feedback)
{
  if (gamification_feedback_pub_ == nullptr) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = feedback.trimmed().toUpper().toStdString();
  gamification_feedback_pub_->publish(msg);
  RCLCPP_INFO(
    node_->get_logger(),
    "Published %s='%s'.",
    kGamificationFeedbackTopic,
    msg.data.c_str());
}

void MainWindow::publishGamificationMode(const QString & mode)
{
  if (gamification_mode_pub_ == nullptr) {
    return;
  }

  const QString cleaned_mode = mode.trimmed().toUpper() == "B" ? "MODE_B" : "MODE_A";
  current_wordle_mode_ =
    cleaned_mode == "MODE_B" ? WordleGameMode::ModeB : WordleGameMode::ModeA;
  if (voice_recording_ || voice_transcribing_) {
    sendVoiceHelperCommand(QStringLiteral("cancel_recording"));
  }
  voice_recording_ = false;
  voice_transcribing_ = false;
  voice_result_pending_ = false;
  pending_voice_guess_.clear();
  voice_retry_rejections_ = 0;
  resetVoicePreview(
    current_wordle_mode_ == WordleGameMode::ModeB ?
    tr("Awaiting input...") : tr("Mode B voice input only"));

  std_msgs::msg::String msg;
  msg.data = cleaned_mode.toStdString();
  gamification_mode_pub_->publish(msg);
  updateVoiceControlsState();
  RCLCPP_INFO(
    node_->get_logger(),
    "Published %s='%s'.",
    kGamificationModeTopic,
    msg.data.c_str());
}

void MainWindow::publishGamificationSecretWord(const QString & word)
{
  if (gamification_secret_word_pub_ == nullptr) {
    return;
  }

  if (!isValidWordleWord(word)) {
    QMessageBox::warning(
      this,
      tr("Invalid Secret"),
      tr("Choose a valid five-letter Wordle word from the dictionary."));
    return;
  }

  std_msgs::msg::String msg;
  msg.data = word.trimmed().toUpper().toStdString();
  gamification_secret_word_pub_->publish(msg);
  RCLCPP_INFO(
    node_->get_logger(),
    "Published %s='<hidden>'.",
    kGamificationSecretWordTopic);
}

void MainWindow::publishGamificationPlayerGuess(const QString & guess)
{
  if (gamification_player_guess_pub_ == nullptr) {
    return;
  }

  if (current_wordle_mode_ != WordleGameMode::ModeB) {
    setVoicePreviewText(tr("Select Mode B first"));
    updateVoiceControlsState();
    return;
  }

  if (!isValidWordleWord(guess)) {
    QMessageBox::warning(
      this,
      tr("Invalid Guess"),
      tr("Enter a valid five-letter Wordle word from the dictionary."));
    return;
  }

  wordle_view_->previewGuess(guess);

  std_msgs::msg::String msg;
  msg.data = guess.trimmed().toUpper().toStdString();
  gamification_player_guess_pub_->publish(msg);
  RCLCPP_INFO(
    node_->get_logger(),
    "Published %s='%s'.",
    kGamificationPlayerGuessTopic,
    msg.data.c_str());
}

void MainWindow::resetGamificationGame()
{
  if (voice_recording_ || voice_transcribing_) {
    sendVoiceHelperCommand(QStringLiteral("cancel_recording"));
  }

  current_wordle_mode_ = WordleGameMode::Unset;
  voice_recording_ = false;
  voice_transcribing_ = false;
  voice_result_pending_ = false;
  pending_voice_guess_.clear();
  voice_retry_rejections_ = 0;
  resetVoicePreview(tr("Select game mode"));
  updateVoiceControlsState();

  if (mission_state_pub_ != nullptr) {
    publishMissionState("RESET");
  }
}

void MainWindow::updateSafetyBanner(const QString & text, const QString & color_hex)
{
  ui_->safetyLabel->setText(text);
  ui_->safetyLabel->setStyleSheet(QString(
    "QLabel#safetyLabel {"
    "  color: %1;"
    "  font-size: 8pt;"
    "  font-weight: 800;"
    "  letter-spacing: 1.5px;"
    "  background: transparent;"
    "  border: none;"
    "  padding: 0px;"
    "}").arg(color_hex));
}

void MainWindow::setupHelpDialog()
{
  QString help_text =
    tr("WORDLEBOT USER GUIDE\n\n"
       "VIEW NAVIGATION\n"
       "Use the left drawer to switch between:\n"
       "  • Sim View: 3D robot simulation (RViz)\n"
       "  • Task View: HL Control task plan and pick/place digital twin\n"
       "  • Camera View: Live camera feed\n\n"
       "CAMERA MODE\n"
       "When viewing the camera:\n"
       "  • Raw Footage: Direct camera stream from RealSense\n"
       "  • Computer Vision: Processed image with letter detection\n\n"
       "VOICE CONTROL\n"
       "Available in Wordle Mode B only:\n"
       "  • Record: Start voice input for letter guesses\n"
       "  • Confirm: Accept the recognized letter\n"
       "  • Retry: Try voice input again\n\n"
	       "SAFETY CONTROLS\n"
	       "  • START / RESUME: Begin mission or continue from pause point\n"
	       "  • SCAN GAME BOARD: Trigger robot scan sweep and perception scan\n"
	       "  • STOP: Halt all motion immediately\n"
	       "  • HOME: Return robot to home position\n\n"
       "DIAGNOSTICS\n"
       "Access from the drawer to view:\n"
       "  • Current mission progress\n"
       "  • System status (Safety, Perception, Wordle)\n"
       "  • Event logs and detailed JSON data\n\n"
       "WORDLE GAME\n"
       "  • Mode A: Robot guesses letters\n"
       "  • Mode B: You guess via voice or typing\n");

  QMessageBox::information(this, tr("How to Use WordleBot"), help_text);
}

void MainWindow::renderMissionProgress(const QString & payload)
{
  if (payload.isEmpty() || payload == last_mission_progress_payload_) {
    return;
  }

  const QJsonDocument document = QJsonDocument::fromJson(payload.toUtf8());
  if (!document.isObject() || diag_mission_steps_layout_ == nullptr) {
    return;
  }

  const QJsonObject object = document.object();
  last_mission_progress_payload_ = payload;

  diag_mission_title_->setText(object.value("title").toString(tr("Wordle Game Pick and Place")));
  diag_mission_summary_->setText(
    object.value("summary").toString(tr("Awaiting mission progress from coordinator.")));
  const QString progress_state = object.value("state").toString().trimmed().toUpper();
  if (!progress_state.isEmpty()) {
    applyCoordinatorMissionState(progress_state, true);
    updateSafetyControlsState();
  }
  if (diagnostics_mission_json_view_ != nullptr) {
    diagnostics_mission_json_view_->setPlainText(
      QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Indented)));
  }

  while (diag_mission_steps_layout_->count() > 0) {
    QLayoutItem * item = diag_mission_steps_layout_->takeAt(0);
    if (item == nullptr) {
      continue;
    }

    if (QWidget * widget = item->widget()) {
      widget->deleteLater();
    }

    delete item;
  }
  mission_step_widgets_.clear();

  const QJsonArray steps = object.value("steps").toArray();
  for (int index = 0; index < steps.size(); ++index) {
    const QJsonObject step_object = steps.at(index).toObject();
    const QString title = step_object.value("title").toString(tr("Unnamed step"));
    const QString detail = step_object.value("detail").toString();
    const QString status = step_object.value("status").toString(QStringLiteral("pending"));

    auto * item = new QFrame(diag_mission_steps_content_);
    item->setObjectName("missionItem");
    item->setProperty("stepStatus", status);

    auto * item_layout = new QVBoxLayout(item);
    item_layout->setContentsMargins(12, 10, 12, 10);
    item_layout->setSpacing(4);

    auto * step_label = new QLabel(QString("%1. %2").arg(index + 1).arg(title), item);
    step_label->setObjectName("missionStep");
    step_label->setProperty("stepStatus", status);
    step_label->setWordWrap(true);
    item_layout->addWidget(step_label);

    auto * detail_label = new QLabel(detail, item);
    detail_label->setObjectName("missionDetail");
    detail_label->setProperty("stepStatus", status);
    detail_label->setWordWrap(true);
    item_layout->addWidget(detail_label);

    item->style()->unpolish(item);
    item->style()->polish(item);
    step_label->style()->unpolish(step_label);
    step_label->style()->polish(step_label);
    detail_label->style()->unpolish(detail_label);
    detail_label->style()->polish(detail_label);

    diag_mission_steps_layout_->addWidget(item);
    mission_step_widgets_.append({item, step_label, detail_label});
  }

  diag_mission_steps_layout_->addStretch();
  refreshDiagnosticsPanel();
}

void MainWindow::appendDiagnosticsEvent(const QString & message)
{
  if (message.isEmpty()) {
    return;
  }

  const QString stamped =
    QStringLiteral("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss"), message);

  if (!diagnostics_event_entries_.isEmpty() && diagnostics_event_entries_.constLast() == stamped) {
    return;
  }

  diagnostics_event_entries_.append(stamped);
  while (diagnostics_event_entries_.size() > 12) {
    diagnostics_event_entries_.removeFirst();
  }

  if (diagnostics_event_log_ != nullptr) {
    diagnostics_event_log_->setPlainText(diagnostics_event_entries_.join('\n'));
  }
}

MainWindow::SafetyControlMode MainWindow::safetyModeForCoordinatorState(const QString & state) const
{
  if (
    state == "STOPPED" ||
    state == "SAFETY_STOPPED" ||
    state == "PERCEPTION_FAILED" ||
    state == "MOTION_FAILED" ||
    state == "ERROR")
  {
    return SafetyControlMode::Stopped;
  }

  if (state == "HOMING") {
    return SafetyControlMode::Homing;
  }

  if (
    state == "SCANNING" ||
    state == "READY_TO_MOVE" ||
    state == "MOVING" ||
    state == "RECOVERING")
  {
    return SafetyControlMode::Active;
  }

  return SafetyControlMode::Idle;
}

void MainWindow::applyCoordinatorMissionState(const QString & state, bool log_event)
{
  const QString next_state = state.trimmed().toUpper();
  if (next_state.isEmpty()) {
    return;
  }

  const bool mission_state_changed = next_state != coordinator_mission_state_;
  coordinator_mission_state_ = next_state;
  safety_mode_ = safetyModeForCoordinatorState(coordinator_mission_state_);

  if (log_event && mission_state_changed) {
    appendDiagnosticsEvent(tr("Mission state changed to %1").arg(coordinator_mission_state_));
  }
}

void MainWindow::launchDiagnosticsWindow()
{
  // Switch to diagnostics page (index 3)
  ui_->contentStack->setCurrentIndex(3);
}

void MainWindow::refreshDiagnosticsPanel()
{
  if (diagnostics_mission_value_label_ == nullptr) {
    return;
  }

  diagnostics_mission_value_label_->setText(
    tr("%1\n%2")
      .arg(coordinator_mission_state_.isEmpty() ? tr("UNKNOWN") : coordinator_mission_state_)
      .arg(last_mission_progress_payload_.isEmpty() ? tr("Awaiting mission progress") : tr("Progress feed active")));

  diagnostics_safety_value_label_->setText(
    human_detected_ ?
      tr("HUMAN DETECTED\nMotion blocked") :
      tr("CLEAR\n%1").arg(safety_mode_ == SafetyControlMode::Stopped ? tr("Stopped") :
        (safety_mode_ == SafetyControlMode::Homing ? tr("Homing") :
        (safety_mode_ == SafetyControlMode::Active ? tr("Active") : tr("Idle")))));

  diagnostics_perception_value_label_->setText(
    tr("%1\nStatus: %2 | Blocks: %3")
      .arg(current_perception_state_.isEmpty() ? tr("UNKNOWN") : current_perception_state_)
      .arg(current_perception_status_.isEmpty() ? tr("UNKNOWN") : current_perception_status_)
      .arg(current_detection_count_));

  QString wordle_text = current_wordle_status_.isEmpty() ? tr("UNKNOWN") : current_wordle_status_;
  if (current_wordle_attempt_ > 0) {
    wordle_text += tr("\nAttempt %1").arg(current_wordle_attempt_);
  }
  if (!current_wordle_guess_.isEmpty()) {
    wordle_text += tr(" | Guess %1").arg(current_wordle_guess_);
  }
  if (current_candidates_left_ > 0) {
    wordle_text += tr("\nCandidates left: %1").arg(current_candidates_left_);
  }
  diagnostics_wordle_value_label_->setText(wordle_text);

  if (diagnostics_game_json_view_ != nullptr) {
    if (last_gamification_diagnostics_payload_.isEmpty()) {
      diagnostics_game_json_view_->setPlainText(tr("Awaiting /diagnostics payload."));
    } else {
      const QJsonDocument diagnostics_doc =
        QJsonDocument::fromJson(last_gamification_diagnostics_payload_.toUtf8());
      diagnostics_game_json_view_->setPlainText(
        diagnostics_doc.isObject() ?
          QString::fromUtf8(diagnostics_doc.toJson(QJsonDocument::Indented)) :
          last_gamification_diagnostics_payload_);
    }
  }

  if (diagnostics_mission_json_view_ != nullptr && last_mission_progress_payload_.isEmpty()) {
    diagnostics_mission_json_view_->setPlainText(tr("Awaiting /wordle_bot/mission_progress payload."));
  }
  if (diagnostics_event_log_ != nullptr && diagnostics_event_entries_.isEmpty()) {
    diagnostics_event_log_->setPlainText(tr("No diagnostic events yet."));
  }
}


void MainWindow::moveEvent(QMoveEvent * event)
{
  QMainWindow::moveEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);
}
