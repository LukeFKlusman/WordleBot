#include "interaction_execution/main_window.hpp"

#include "interaction_execution/camera_view.hpp"
#include "interaction_execution/rviz_sim_view.hpp"
#include "interaction_execution/wordle_view.hpp"
#include "ui_rs2_concept.h"

#include <algorithm>
#include <QFrame>
#include <QIcon>
#include <QLabel>
#include <QMoveEvent>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QRandomGenerator>
#include <QResizeEvent>
#include <QScrollArea>
#include <QStyle>
#include <QTabBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

namespace
{
const QStringList kWordleDemoWords{
  "apple", "baker", "chair", "droid", "eagle", "flame", "grape", "house",
  "input", "jolly", "kneel", "laser", "mango", "noble", "ocean", "piano",
  "queen", "robot", "smile", "table", "ultra", "vivid", "whale", "xenon",
  "yacht", "zebra"};

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

QString randomWordleWord()
{
  const int index = QRandomGenerator::global()->bounded(kWordleDemoWords.size());
  return kWordleDemoWords.at(index);
}
}  // namespace

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent), ui_(std::make_unique<Ui::MainWindow>()), node_(std::move(node))
{
  ui_->setupUi(this);
  setupTabs();
  setupVoiceControls();
  setupMissionOverlay();
}

MainWindow::~MainWindow() = default;

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

  mission_tab_page_ = new QWidget();
  mission_tab_index_ = ui_->mainTab->addTab(mission_tab_page_, tr("Mission"));
  last_content_tab_index_ = ui_->mainTab->currentIndex();

  connect(ui_->mainTab, &QTabWidget::currentChanged, this, &MainWindow::handleMainTabChanged);
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
    voice_recording_ = true;
    ui_->voiceTranscriptValue->setText(tr("Listening..."));
    updateVoiceControlsState();
  });

  connect(ui_->voiceStopButton, &QPushButton::clicked, this, [this]() {
    if (!voice_recording_) {
      return;
    }

    voice_recording_ = false;
    pending_voice_guess_ = randomWordleWord();
    wordle_view_->previewGuess(pending_voice_guess_);
    ui_->voiceTranscriptValue->setText(tr("Preview ready"));
    updateVoiceControlsState();
  });

  connect(ui_->voiceConfirmButton, &QPushButton::clicked, this, [this]() {
    if (pending_voice_guess_.isEmpty()) {
      return;
    }

    wordle_view_->submitPreviewGuess();
    pending_voice_guess_.clear();
    ui_->voiceTranscriptValue->setText(tr("Guess submitted"));
    updateVoiceControlsState();
  });

  connect(ui_->voiceRetryButton, &QPushButton::clicked, this, [this]() {
    if (pending_voice_guess_.isEmpty()) {
      return;
    }

    wordle_view_->clearPreviewGuess();
    pending_voice_guess_.clear();
    ui_->voiceTranscriptValue->setText(tr("Awaiting input..."));
    updateVoiceControlsState();
  });

  updateVoiceControlsState();
}

void MainWindow::updateVoiceControlsState()
{
  const bool has_pending_guess = !pending_voice_guess_.isEmpty();

  ui_->voiceRecordButton->setEnabled(!voice_recording_ && !has_pending_guess);
  ui_->voiceStopButton->setEnabled(voice_recording_);
  ui_->voiceConfirmButton->setEnabled(has_pending_guess);
  ui_->voiceRetryButton->setEnabled(has_pending_guess);
}

void MainWindow::setupMissionOverlay()
{
  mission_overlay_ = new QFrame(this, Qt::Tool | Qt::FramelessWindowHint);
  mission_overlay_->setObjectName("missionOverlay");
  mission_overlay_->setAttribute(Qt::WA_StyledBackground, true);
  mission_overlay_->setAttribute(Qt::WA_ShowWithoutActivating, true);
  mission_overlay_->setFocusPolicy(Qt::NoFocus);
  mission_overlay_->setStyleSheet(
    "QFrame#missionOverlay {"
    "  background-color: rgba(13, 16, 24, 235);"
    "  border-left: 1px solid rgba(165, 180, 252, 0.18);"
    "}"
    "QLabel#missionHeader {"
    "  color: #e2e8f0;"
    "  font-size: 13pt;"
    "  font-weight: 700;"
    "  letter-spacing: 1px;"
    "}"
    "QLabel#missionSubheader {"
    "  color: #94a3b8;"
    "  font-size: 8pt;"
    "  font-weight: 600;"
    "  letter-spacing: 1px;"
    "}"
    "QFrame#missionItem {"
    "  background-color: rgba(30, 41, 59, 0.92);"
    "  border: 1px solid rgba(148, 163, 184, 0.18);"
    "  border-radius: 8px;"
    "}"
    "QLabel#missionStep {"
    "  color: #f8fafc;"
    "  font-size: 10pt;"
    "  font-weight: 600;"
    "}"
    "QLabel#missionDetail {"
    "  color: #94a3b8;"
    "  font-size: 8.5pt;"
    "}");

  auto * overlay_layout = new QVBoxLayout(mission_overlay_);
  overlay_layout->setContentsMargins(18, 18, 18, 18);
  overlay_layout->setSpacing(12);

  auto * header = new QLabel(tr("CURRENT MISSION"), mission_overlay_);
  header->setObjectName("missionSubheader");
  overlay_layout->addWidget(header);

  auto * title = new QLabel(tr("Wordle Game Pick and Place"), mission_overlay_);
  title->setObjectName("missionHeader");
  title->setWordWrap(true);
  overlay_layout->addWidget(title);

  auto * scroll = new QScrollArea(mission_overlay_);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setStyleSheet(
    "QScrollArea { background: transparent; }"
    "QScrollArea > QWidget > QWidget { background: transparent; }"
    "QScrollBar:vertical { background: transparent; width: 8px; }"
    "QScrollBar::handle:vertical { background: rgba(148, 163, 184, 0.35); border-radius: 4px; }"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }");

  auto * scroll_content = new QWidget(scroll);
  auto * scroll_layout = new QVBoxLayout(scroll_content);
  scroll_layout->setContentsMargins(0, 0, 0, 0);
  scroll_layout->setSpacing(10);

  const QStringList mission_steps{
    tr("1. Confirm active puzzle state"),
    tr("2. Capture overhead camera snapshot"),
    tr("3. Move arm to pre-grasp waypoint"),
    tr("4. Pick letter block from tray"),
    tr("5. Align over target board slot"),
    tr("6. Place block and verify pose"),
    tr("7. Repeat for remaining letters"),
    tr("8. Return manipulator to safe home")};

  const QStringList mission_details{
    tr("Read the current board and validate the next candidate word before motion starts."),
    tr("Trigger the camera pipeline and freeze the latest frame for the operator preview."),
    tr("Lift away from the board and clear the camera view before approaching the tray."),
    tr("Close the gripper on the selected block once the target letter is confirmed."),
    tr("Use the board pose estimate to center above the next empty square."),
    tr("Open the gripper, wait for contact settle, and check placement tolerance."),
    tr("Continue until the entire five-letter guess has been staged on the board."),
    tr("Withdraw vertically, then move to the default idle pose and await confirmation.")};

  for (int index = 0; index < mission_steps.size(); ++index) {
    auto * item = new QFrame(scroll_content);
    item->setObjectName("missionItem");

    auto * item_layout = new QVBoxLayout(item);
    item_layout->setContentsMargins(12, 10, 12, 10);
    item_layout->setSpacing(4);

    auto * step = new QLabel(mission_steps.at(index), item);
    step->setObjectName("missionStep");
    step->setWordWrap(true);
    item_layout->addWidget(step);

    auto * detail = new QLabel(mission_details.at(index), item);
    detail->setObjectName("missionDetail");
    detail->setWordWrap(true);
    item_layout->addWidget(detail);

    scroll_layout->addWidget(item);
  }

  scroll_layout->addStretch();
  scroll->setWidget(scroll_content);
  overlay_layout->addWidget(scroll);

  mission_overlay_->hide();
  syncMissionOverlayGeometry();
  updateMissionTabAppearance();
}

void MainWindow::handleMainTabChanged(int index)
{
  if (restoring_content_tab_) {
    return;
  }

  if (index == mission_tab_index_) {
    toggleMissionOverlay();

    restoring_content_tab_ = true;
    ui_->mainTab->setCurrentIndex(last_content_tab_index_);
    restoring_content_tab_ = false;
    return;
  }

  last_content_tab_index_ = index;
  syncMissionOverlayGeometry();
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

  syncMissionOverlayGeometry();
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

void MainWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);
  QTimer::singleShot(0, this, [this]() {
    syncMissionOverlayGeometry();
  });
}

void MainWindow::moveEvent(QMoveEvent * event)
{
  QMainWindow::moveEvent(event);
  QTimer::singleShot(0, this, [this]() {
    syncMissionOverlayGeometry();
  });
}
