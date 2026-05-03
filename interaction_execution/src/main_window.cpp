#include "interaction_execution/main_window.hpp"

#include "interaction_execution/camera_view.hpp"
#include "interaction_execution/rviz_sim_view.hpp"
#include "interaction_execution/wordle_view.hpp"
#include "ui_rs2_concept.h"

#include <algorithm>
#include <QFile>
#include <QFileInfo>
#include <QCloseEvent>
#include <QDateTime>
#include <QEvent>
#include <QFrame>
#include <QFontDatabase>
#include <QGridLayout>
#include <QIcon>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLayoutItem>
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

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
constexpr const char * kPerceptionStateTopic = "/mission/state";
constexpr const char * kCoordinatorMissionStateTopic = "/wordle_bot/mission_state";
constexpr const char * kMissionProgressTopic = "/wordle_bot/mission_progress";
constexpr const char * kMissionCommandTopic = "/wordle_bot/mission_cmd";
constexpr const char * kHumanDetectedTopic = "/perception/human_detected";
constexpr const char * kPerceptionStatusTopic = "/perception/status";
constexpr const char * kPerceptionDetectionsTopic = "/perception/detections";
constexpr const char * kGamificationGuessTopic = "/gamification/guess";
constexpr const char * kGamificationFeedbackTopic = "/gamification/feedback";
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

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent), ui_(std::make_unique<Ui::MainWindow>()), node_(std::move(node))
{
  ui_->setupUi(this);
  setupVisualDesign();
  loadWordleDictionary();
  setupTabs();
  setupGamificationBridge();
  setupVoiceControls();
  setupVoiceHelper();
  setupSafetyControls();
  reserveSidebarWidth();
  setupDiagnosticsWindow();
  setupMissionOverlay();
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
  auto * tab_bar = ui_ != nullptr && ui_->mainTab != nullptr ? ui_->mainTab->tabBar() : nullptr;
  if (watched == tab_bar && tab_bar != nullptr) {
    if (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseButtonRelease) {
      auto * mouse_event = static_cast<QMouseEvent *>(event);
      const int clicked_tab = tab_bar->tabAt(mouse_event->pos());
      if (clicked_tab == mission_tab_index_) {
        if (event->type() == QEvent::MouseButtonRelease &&
          mouse_event->button() == Qt::LeftButton)
        {
          toggleMissionOverlay();
        }
        return true;
      }
      if (clicked_tab == diagnostics_tab_index_) {
        if (event->type() == QEvent::MouseButtonRelease &&
          mouse_event->button() == Qt::LeftButton)
        {
          launchDiagnosticsWindow();
        }
        return true;
      }
    }
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::Close) {
    auto * close_event = static_cast<QCloseEvent *>(event);
    close_event->ignore();
    diagnostics_window_->hide();
    updateDiagnosticsTabAppearance();
    return true;
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::WindowActivate) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::Hide) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::Show) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::FocusIn) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::FocusOut) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::WindowDeactivate) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::Move) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::Resize) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::ZOrderChange) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::ActivationChange) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::WinIdChange) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::ParentChange) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::PlatformSurface) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::Expose) {
    updateDiagnosticsTabAppearance();
  }

  if (watched == diagnostics_window_ && event->type() == QEvent::Paint) {
    if (diagnostics_window_->isVisible()) {
      updateDiagnosticsTabAppearance();
    }
  }

  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::setupTabs()
{
  auto * sim_layout = new QVBoxLayout(ui_->tab);
  sim_layout->setContentsMargins(0, 0, 0, 0);
  rviz_view_ = new RvizSimView(node_, ui_->tab);
  sim_layout->addWidget(rviz_view_);

  auto * camera_layout = new QVBoxLayout(ui_->tab_2);
  camera_layout->setContentsMargins(0, 0, 0, 0);
  camera_view_ = new CameraView(node_, ui_->tab_2);
  camera_layout->addWidget(camera_view_);

  auto * wordle_layout = new QVBoxLayout(ui_->wordle);
  wordle_layout->setContentsMargins(0, 0, 0, 0);
  wordle_view_ = new WordleView(ui_->wordle);
  wordle_layout->addWidget(wordle_view_);

  diagnostics_tab_index_ = ui_->mainTab->indexOf(ui_->tab_3);
  ui_->mainTab->setTabIcon(diagnostics_tab_index_, makeLaunchIcon());
  ui_->mainTab->setIconSize(QSize(14, 14));

  mission_tab_page_ = new QWidget();
  mission_tab_index_ = ui_->mainTab->addTab(mission_tab_page_, tr("Mission"));
  last_content_tab_index_ = ui_->mainTab->currentIndex();

  ui_->mainTab->tabBar()->installEventFilter(this);
  connect(ui_->mainTab, &QTabWidget::currentChanged, this, &MainWindow::handleMainTabChanged);
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

  ui_->mainTab->setStyleSheet(
    "QTabWidget {"
    "  background-color: #111820;"
    "}"
    "QTabWidget::pane {"
    "  background-color: #0f151c;"
    "  border: none;"
    "  border-top: 1px solid rgba(139, 148, 158, 0.18);"
    "  margin: 0px;"
    "  padding: 0px;"
    "}"
    "QWidget#tab,"
    "QWidget#tab_2,"
    "QWidget#tab_3 {"
    "  background-color: #0f151c;"
    "}"
    "QTabWidget::tab-bar {"
    "  alignment: left;"
    "}"
    "QTabBar {"
    "  background-color: #111820;"
    "}"
    "QTabBar::tab {"
    "  background-color: transparent;"
    "  color: #8b949e;"
    "  padding: 11px 22px;"
    "  font-size: 9pt;"
    "  font-weight: 700;"
    "  border: none;"
    "  border-bottom: 3px solid transparent;"
    "  min-width: 92px;"
    "}"
    "QTabBar::tab:selected {"
    "  color: #f0f6fc;"
    "  border-bottom-color: #f2cc60;"
    "  background-color: #161f29;"
    "}"
    "QTabBar::tab:hover:!selected {"
    "  color: #c9d1d9;"
    "  background-color: rgba(240, 246, 252, 0.05);"
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
  diagnostics_window_ = new QWidget(nullptr, Qt::Window);
  diagnostics_window_->setAttribute(Qt::WA_DeleteOnClose, false);
  diagnostics_window_->setWindowTitle(tr("Diagnostics"));
  diagnostics_window_->installEventFilter(this);
  diagnostics_window_->setStyleSheet(
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

  auto * root_layout = new QVBoxLayout(diagnostics_window_);
  root_layout->setContentsMargins(18, 18, 18, 18);
  root_layout->setSpacing(14);

  auto * summary_label = new QLabel(tr("SYSTEM SUMMARY"), diagnostics_window_);
  summary_label->setObjectName("diagnosticsSectionLabel");
  root_layout->addWidget(summary_label);

  auto * cards_layout = new QGridLayout();
  cards_layout->setHorizontalSpacing(12);
  cards_layout->setVerticalSpacing(12);

  const auto create_card = [this](const QString & title, QLabel ** value_label) {
    auto * card = new QFrame(diagnostics_window_);
    card->setObjectName("diagnosticsCard");

    auto * layout = new QVBoxLayout(card);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(6);

    auto * title_label = new QLabel(title, card);
    title_label->setObjectName("diagnosticsCardTitle");
    layout->addWidget(title_label);

    auto * value = new QLabel(tr("Waiting for data"), card);
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

  auto * events_label = new QLabel(tr("RECENT EVENTS"), diagnostics_window_);
  events_label->setObjectName("diagnosticsSectionLabel");
  lower_layout->addWidget(events_label, 0, 0);

  diagnostics_event_log_ = new QPlainTextEdit(diagnostics_window_);
  diagnostics_event_log_->setObjectName("diagnosticsTextPane");
  diagnostics_event_log_->setReadOnly(true);
  diagnostics_event_log_->setMinimumHeight(180);
  diagnostics_event_log_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  lower_layout->addWidget(diagnostics_event_log_, 1, 0);

  auto * mission_json_label = new QLabel(tr("MISSION PROGRESS JSON"), diagnostics_window_);
  mission_json_label->setObjectName("diagnosticsSectionLabel");
  lower_layout->addWidget(mission_json_label, 0, 1);

  diagnostics_mission_json_view_ = new QPlainTextEdit(diagnostics_window_);
  diagnostics_mission_json_view_->setObjectName("diagnosticsTextPane");
  diagnostics_mission_json_view_->setReadOnly(true);
  diagnostics_mission_json_view_->setMinimumHeight(180);
  diagnostics_mission_json_view_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  lower_layout->addWidget(diagnostics_mission_json_view_, 1, 1);

  auto * game_json_label = new QLabel(tr("WORDLE DIAGNOSTICS JSON"), diagnostics_window_);
  game_json_label->setObjectName("diagnosticsSectionLabel");
  lower_layout->addWidget(game_json_label, 2, 0, 1, 2);

  diagnostics_game_json_view_ = new QPlainTextEdit(diagnostics_window_);
  diagnostics_game_json_view_->setObjectName("diagnosticsTextPane");
  diagnostics_game_json_view_->setReadOnly(true);
  diagnostics_game_json_view_->setMinimumHeight(180);
  diagnostics_game_json_view_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  lower_layout->addWidget(diagnostics_game_json_view_, 3, 0, 1, 2);

  root_layout->addLayout(lower_layout);
  root_layout->addStretch();

  refreshDiagnosticsPanel();
  diagnostics_window_->resize(620, 760);
  diagnostics_window_->hide();
  updateDiagnosticsTabAppearance();
}

void MainWindow::setupVoiceControls()
{
  ui_->voiceRecordButton->setIcon(makeRecordIcon());
  ui_->voiceRecordButton->setIconSize(QSize(18, 18));
  ui_->voiceStopButton->setIcon(makeStopIcon());
  ui_->voiceStopButton->setIconSize(QSize(18, 18));
  ui_->voiceConfirmButton->setIcon(makeConfirmIcon());
  ui_->voiceConfirmButton->setIconSize(QSize(18, 18));
  ui_->voiceRetryButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  ui_->voiceRetryButton->setIconSize(QSize(18, 18));

  connect(ui_->voiceRecordButton, &QPushButton::clicked, this, [this]() {
    if (!voice_helper_available_ || voice_helper_process_ == nullptr) {
      return;
    }

    wordle_view_->clearPreviewGuess();
    pending_voice_guess_.clear();
    voice_recording_ = true;
    voice_transcribing_ = false;
    setVoicePreviewText(tr("Listening..."));
    sendVoiceHelperCommand(QStringLiteral("start_recording"));
    updateVoiceControlsState();
  });

  connect(ui_->voiceStopButton, &QPushButton::clicked, this, [this]() {
    if (!voice_recording_ || voice_helper_process_ == nullptr) {
      return;
    }

    voice_recording_ = false;
    voice_transcribing_ = true;
    setVoicePreviewText(tr("Transcribing..."));
    sendVoiceHelperCommand(QStringLiteral("stop_recording"));
    updateVoiceControlsState();
  });

  connect(ui_->voiceConfirmButton, &QPushButton::clicked, this, [this]() {
    if (pending_voice_guess_.isEmpty()) {
      return;
    }

    wordle_view_->submitPreviewGuess();
    setVoicePreviewText(tr("Confirmed: %1").arg(pending_voice_guess_.toUpper()));
    pending_voice_guess_.clear();
    updateVoiceControlsState();
  });

  connect(ui_->voiceRetryButton, &QPushButton::clicked, this, [this]() {
    if (voice_recording_ || voice_transcribing_) {
      sendVoiceHelperCommand(QStringLiteral("cancel_recording"));
    }

    voice_recording_ = false;
    voice_transcribing_ = false;
    wordle_view_->clearPreviewGuess();
    pending_voice_guess_.clear();
    resetVoicePreview();
    updateVoiceControlsState();
  });

  resetVoicePreview();
  updateVoiceControlsState();
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
  resetVoicePreview();
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
    resetVoicePreview();
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
    pending_voice_guess_.clear();
    wordle_view_->clearPreviewGuess();
    resetVoicePreview();
    updateVoiceControlsState();
    return;
  }

  if (event == "recording_result") {
    voice_recording_ = false;
    voice_transcribing_ = false;

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

  connect(wordle_view_, &WordleView::feedbackSubmitted, this, [this](const QString & feedback) {
    publishGamificationFeedback(feedback);
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
      const int attempt = object.value("attempt").toInt(0);
      const QString guess = object.value("current_guess").toString();
      current_wordle_status_ = status;
      current_wordle_attempt_ = attempt;
      current_wordle_guess_ = guess;
      current_candidates_left_ = object.value("candidates_left").toInt(0);

      QString summary = tr("Status: %1").arg(status);
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
  const bool can_record = voice_helper_available_ && !voice_recording_ && !voice_transcribing_;
  const bool can_stop = voice_helper_available_ && voice_recording_;
  const bool can_confirm =
    voice_helper_available_ && !voice_recording_ && !voice_transcribing_ &&
    !pending_voice_guess_.isEmpty();
  const bool can_retry =
    voice_helper_available_ &&
    (voice_recording_ || voice_transcribing_ || !pending_voice_guess_.isEmpty() ||
    voice_preview_text_ != QStringLiteral("Awaiting input..."));

  ui_->voiceRecordButton->setEnabled(can_record);
  ui_->voiceStopButton->setEnabled(can_stop);
  ui_->voiceConfirmButton->setEnabled(can_confirm);
  ui_->voiceRetryButton->setEnabled(can_retry);
}

void MainWindow::setupSafetyControls()
{
  mission_state_pub_ = node_->create_publisher<std_msgs::msg::String>(kPerceptionStateTopic, 10);
  mission_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>(kMissionCommandTopic, 10);
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

      const QString next_state = QString::fromStdString(msg->data).trimmed().toUpper();
      const bool mission_state_changed = next_state != coordinator_mission_state_;
      coordinator_mission_state_ = next_state;
      if (coordinator_mission_state_ == "STOPPED") {
        safety_mode_ = SafetyControlMode::Stopped;
      } else if (coordinator_mission_state_ == "HOMING") {
        safety_mode_ = SafetyControlMode::Homing;
      } else if (
        coordinator_mission_state_ == "SCANNING" ||
        coordinator_mission_state_ == "READY_TO_MOVE" ||
        coordinator_mission_state_ == "MOVING")
      {
        safety_mode_ = SafetyControlMode::Active;
      } else {
        safety_mode_ = SafetyControlMode::Idle;
      }

      if (mission_state_changed) {
        appendDiagnosticsEvent(tr("Mission state changed to %1").arg(coordinator_mission_state_));
      }
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
      coordinator_mission_state_ = "STOPPED";
      safety_mode_ = SafetyControlMode::Stopped;
      appendDiagnosticsEvent(tr("Safety stop triggered by human detection"));
      updateSafetyControlsState();
      refreshDiagnosticsPanel();
    });

  connect(ui_->pushButton, &QPushButton::clicked, this, [this]() {
    if (safety_mode_ != SafetyControlMode::Idle) {
      return;
    }

    if (human_detected_) {
      RCLCPP_WARN(node_->get_logger(), "START blocked because a human is currently detected.");
      updateSafetyControlsState();
      return;
    }

    publishMissionState("SCANNING");
    publishMissionCommand("START");
    coordinator_mission_state_ = "SCANNING";
    safety_mode_ = SafetyControlMode::Active;
    appendDiagnosticsEvent(tr("Operator command: START"));
    updateSafetyControlsState();
    refreshDiagnosticsPanel();
  });

  connect(ui_->pushButton_4, &QPushButton::clicked, this, [this]() {
    if (safety_mode_ != SafetyControlMode::Active && safety_mode_ != SafetyControlMode::Homing) {
      return;
    }

    publishMissionState("IDLE");
    publishMissionCommand("STOP");
    coordinator_mission_state_ = "STOPPED";
    safety_mode_ = SafetyControlMode::Stopped;
    appendDiagnosticsEvent(tr("Operator command: STOP"));
    updateSafetyControlsState();
    refreshDiagnosticsPanel();
  });

  connect(ui_->pushButton_2, &QPushButton::clicked, this, [this]() {
    if (safety_mode_ != SafetyControlMode::Stopped) {
      return;
    }

    if (human_detected_) {
      RCLCPP_WARN(node_->get_logger(), "RESUME blocked because a human is currently detected.");
      updateSafetyControlsState();
      return;
    }

    publishMissionState("SCANNING");
    publishMissionCommand("RESUME");
    coordinator_mission_state_ = "SCANNING";
    safety_mode_ = SafetyControlMode::Active;
    appendDiagnosticsEvent(tr("Operator command: RESUME"));
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

    publishMissionState("SCANNING");
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
  const bool can_start = !human_detected_ && safety_mode_ == SafetyControlMode::Idle;
  const bool can_stop =
    safety_mode_ == SafetyControlMode::Active || safety_mode_ == SafetyControlMode::Homing;
  const bool can_resume = !human_detected_ && safety_mode_ == SafetyControlMode::Stopped;
  const bool can_home = !human_detected_ && safety_mode_ == SafetyControlMode::Stopped;

  ui_->pushButton->setEnabled(can_start);
  ui_->pushButton_4->setEnabled(can_stop);
  ui_->pushButton_2->setEnabled(can_resume);
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
      updateSafetyBanner("SAFETY CONTROLS | STOPPED", "#f85149");
      return;
    case SafetyControlMode::Homing:
      updateSafetyBanner("SAFETY CONTROLS | RETURN HOME", "#f2cc60");
      return;
    case SafetyControlMode::Active:
      if (coordinator_mission_state_ == "MOVING" || coordinator_mission_state_ == "READY_TO_MOVE") {
        updateSafetyBanner("SAFETY CONTROLS | ACTIVE", "#56d364");
      } else {
        updateSafetyBanner("SAFETY CONTROLS | SCANNING", "#56d364");
      }
      return;
  }
}

void MainWindow::reserveSidebarWidth()
{
  const QString widest_safety_text = tr("SAFETY CONTROLS | RETURN HOME");
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

void MainWindow::setupMissionOverlay()
{
  mission_overlay_ = new QFrame(this, Qt::Tool | Qt::FramelessWindowHint | Qt::BypassWindowManagerHint);
  mission_overlay_->setObjectName("missionOverlay");
  mission_overlay_->setAttribute(Qt::WA_StyledBackground, true);
  mission_overlay_->setAttribute(Qt::WA_ShowWithoutActivating, true);
  mission_overlay_->setFocusPolicy(Qt::StrongFocus);
  mission_overlay_->setStyleSheet(
    "QFrame#missionOverlay {"
    "  background-color: rgba(11, 15, 20, 242);"
    "  border-left: 1px solid rgba(139, 148, 158, 0.24);"
    "}"
    "QLabel#missionHeader {"
    "  color: #f0f6fc;"
    "  font-size: 13pt;"
    "  font-weight: 800;"
    "  letter-spacing: 0px;"
    "}"
    "QLabel#missionSubheader {"
    "  color: #79c0ff;"
    "  font-size: 8pt;"
    "  font-weight: 800;"
    "  letter-spacing: 1.5px;"
    "}"
    "QLabel#missionSummary {"
    "  color: #c9d1d9;"
    "  font-size: 8.5pt;"
    "  line-height: 1.35;"
    "}"
    "QFrame#missionItem {"
    "  background-color: #111820;"
    "  border: 1px solid rgba(139, 148, 158, 0.18);"
    "  border-radius: 8px;"
    "}"
    "QFrame#missionItem[stepStatus=\"active\"] {"
    "  background-color: rgba(31, 111, 235, 0.22);"
    "  border: 1px solid rgba(121, 192, 255, 0.42);"
    "}"
    "QFrame#missionItem[stepStatus=\"done\"] {"
    "  background-color: rgba(35, 134, 54, 0.22);"
    "  border: 1px solid rgba(86, 211, 100, 0.34);"
    "}"
    "QFrame#missionItem[stepStatus=\"blocked\"] {"
    "  background-color: rgba(218, 54, 51, 0.22);"
    "  border: 1px solid rgba(248, 81, 73, 0.40);"
    "}"
    "QLabel#missionStep {"
    "  color: #f0f6fc;"
    "  font-size: 10pt;"
    "  font-weight: 800;"
    "}"
    "QLabel#missionStep[stepStatus=\"done\"] {"
    "  color: #aff5b4;"
    "}"
    "QLabel#missionStep[stepStatus=\"blocked\"] {"
    "  color: #ffdcd7;"
    "}"
    "QLabel#missionDetail {"
    "  color: #8b949e;"
    "  font-size: 8.5pt;"
    "}"
    "QLabel#missionDetail[stepStatus=\"active\"] {"
    "  color: #cae8ff;"
    "}"
    "QLabel#missionDetail[stepStatus=\"done\"] {"
    "  color: #dcffe4;"
    "}"
    "QLabel#missionDetail[stepStatus=\"blocked\"] {"
    "  color: #ffdcd7;"
    "}");

  auto * overlay_layout = new QVBoxLayout(mission_overlay_);
  overlay_layout->setContentsMargins(18, 18, 18, 18);
  overlay_layout->setSpacing(12);

  auto * header = new QLabel(tr("CURRENT MISSION"), mission_overlay_);
  header->setObjectName("missionSubheader");
  overlay_layout->addWidget(header);

  mission_title_label_ = new QLabel(tr("Wordle Game Pick and Place"), mission_overlay_);
  mission_title_label_->setObjectName("missionHeader");
  mission_title_label_->setWordWrap(true);
  overlay_layout->addWidget(mission_title_label_);

  mission_summary_label_ = new QLabel(tr("Awaiting mission progress from coordinator."), mission_overlay_);
  mission_summary_label_->setObjectName("missionSummary");
  mission_summary_label_->setWordWrap(true);
  overlay_layout->addWidget(mission_summary_label_);

  auto * scroll = new QScrollArea(mission_overlay_);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setStyleSheet(
    "QScrollArea { background: transparent; }"
    "QScrollArea > QWidget > QWidget { background: transparent; }"
    "QScrollBar:vertical { background: transparent; width: 8px; }"
    "QScrollBar::handle:vertical { background: rgba(139, 148, 158, 0.40); border-radius: 4px; }"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }");

  mission_steps_content_ = new QWidget(scroll);
  mission_steps_layout_ = new QVBoxLayout(mission_steps_content_);
  mission_steps_layout_->setContentsMargins(0, 0, 0, 0);
  mission_steps_layout_->setSpacing(10);
  mission_steps_layout_->addStretch();
  scroll->setWidget(mission_steps_content_);
  overlay_layout->addWidget(scroll);

  mission_progress_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kMissionProgressTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg == nullptr) {
        return;
      }

      renderMissionProgress(QString::fromStdString(msg->data));
    });

  renderMissionProgress(QStringLiteral(
    "{\"title\":\"Wordle Game Pick and Place\",\"summary\":\"Awaiting mission progress from coordinator.\",\"steps\":[]}"));

  mission_overlay_->hide();
  syncMissionOverlayGeometry();
  updateMissionTabAppearance();
}

void MainWindow::renderMissionProgress(const QString & payload)
{
  if (payload.isEmpty() || payload == last_mission_progress_payload_) {
    return;
  }

  const QJsonDocument document = QJsonDocument::fromJson(payload.toUtf8());
  if (!document.isObject() || mission_steps_layout_ == nullptr) {
    return;
  }

  const QJsonObject object = document.object();
  last_mission_progress_payload_ = payload;

  mission_title_label_->setText(object.value("title").toString(tr("Wordle Game Pick and Place")));
  mission_summary_label_->setText(
    object.value("summary").toString(tr("Awaiting mission progress from coordinator.")));
  if (diagnostics_mission_json_view_ != nullptr) {
    diagnostics_mission_json_view_->setPlainText(
      QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Indented)));
  }

  while (mission_steps_layout_->count() > 0) {
    QLayoutItem * item = mission_steps_layout_->takeAt(0);
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

    auto * item = new QFrame(mission_steps_content_);
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

    mission_steps_layout_->addWidget(item);
    mission_step_widgets_.append({item, step_label, detail_label});
  }

  mission_steps_layout_->addStretch();
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

void MainWindow::launchDiagnosticsWindow()
{
  if (diagnostics_window_ == nullptr) {
    return;
  }

  if (!diagnostics_window_->isVisible()) {
    diagnostics_window_->move(frameGeometry().topRight() + QPoint(16, 0));
    diagnostics_window_->show();
  } else if (diagnostics_window_->isMinimized()) {
    diagnostics_window_->showNormal();
  }

  diagnostics_window_->raise();
  diagnostics_window_->activateWindow();
  updateDiagnosticsTabAppearance();
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

void MainWindow::updateDiagnosticsTabAppearance()
{
  auto * tab_bar = ui_->mainTab->tabBar();
  if (tab_bar == nullptr || diagnostics_tab_index_ < 0) {
    return;
  }

  const bool window_visible = diagnostics_window_ != nullptr && diagnostics_window_->isVisible();
  tab_bar->setTabTextColor(
    diagnostics_tab_index_,
    window_visible ? QColor("#67e8f9") : QColor("#64748b"));
}

void MainWindow::handleMainTabChanged(int index)
{
  if (index == diagnostics_tab_index_) {
    QTimer::singleShot(0, this, [this]() {
      if (last_content_tab_index_ >= 0) {
        ui_->mainTab->setCurrentIndex(last_content_tab_index_);
        launchDiagnosticsWindow();
      }
    });
    return;
  }

  if (index == mission_tab_index_) {
    QTimer::singleShot(0, this, [this]() {
      if (last_content_tab_index_ >= 0) {
        ui_->mainTab->setCurrentIndex(last_content_tab_index_);
      }
    });
    return;
  }

  last_content_tab_index_ = index;
  QTimer::singleShot(0, this, [this]() {
    syncMissionOverlayGeometry();
  });
}

void MainWindow::toggleMissionOverlay()
{
  if (mission_overlay_ == nullptr) {
    return;
  }

  if (mission_overlay_->isVisible()) {
    mission_overlay_->hide();
  } else {
    syncMissionOverlayGeometry();
    mission_overlay_->show();
    mission_overlay_->raise();
  }

  updateMissionTabAppearance();
}

void MainWindow::syncMissionOverlayGeometry()
{
  if (mission_overlay_ == nullptr || last_content_tab_index_ < 0) {
    return;
  }

  QWidget * content_page = ui_->mainTab->widget(last_content_tab_index_);
  if (content_page == nullptr) {
    return;
  }

  const QRect content_rect = content_page->rect();
  const int overlay_width = std::clamp(content_rect.width() / 3, 280, 360);
  const QPoint overlay_top_left = content_page->mapToGlobal(
    QPoint(content_rect.width() - overlay_width, 0));

  mission_overlay_->setGeometry(
    overlay_top_left.x(),
    overlay_top_left.y(),
    overlay_width,
    content_rect.height());

  if (mission_overlay_->isVisible()) {
    mission_overlay_->raise();
  }
}

void MainWindow::updateMissionTabAppearance()
{
  auto * tab_bar = ui_->mainTab->tabBar();
  if (tab_bar == nullptr || mission_tab_index_ < 0) {
    return;
  }

  tab_bar->setTabTextColor(
    mission_tab_index_,
    mission_overlay_ != nullptr && mission_overlay_->isVisible() ?
      QColor("#a5b4fc") : QColor("#64748b"));
}

void MainWindow::moveEvent(QMoveEvent * event)
{
  QMainWindow::moveEvent(event);
  QTimer::singleShot(0, this, [this]() {
    syncMissionOverlayGeometry();
  });
}

void MainWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);
  QTimer::singleShot(0, this, [this]() {
    syncMissionOverlayGeometry();
  });
}
