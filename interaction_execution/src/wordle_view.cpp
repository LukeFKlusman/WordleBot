#include "interaction_execution/wordle_view.hpp"

#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>

#ifdef HAS_QT_WEBENGINE
#include <QUrl>
#include <QtWebEngineWidgets/QWebEngineView>
#endif

#include <algorithm>
#include <exception>
#include <functional>

#ifdef HAS_QT_WEBENGINE
namespace
{
class WordleWebView : public QWebEngineView
{
public:
  using QWebEngineView::QWebEngineView;

  std::function<bool(QKeyEvent *)> key_handler;

protected:
  void keyPressEvent(QKeyEvent * event) override
  {
    if (key_handler && key_handler(event)) {
      event->accept();
      return;
    }

    QWebEngineView::keyPressEvent(event);
  }

  void mousePressEvent(QMouseEvent * event) override
  {
    setFocus(Qt::MouseFocusReason);
    QWebEngineView::mousePressEvent(event);
  }
};
}  // namespace
#endif

WordleView::WordleView(QWidget * parent)
: QWidget(parent)
{
  setFocusPolicy(Qt::StrongFocus);
  qApp->installEventFilter(this);

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  QString status_message;

  try {
    const auto package_share_dir =
      QString::fromStdString(ament_index_cpp::get_package_share_directory("interaction_execution"));
    const auto wordle_path = package_share_dir + "/wordle-clone/index.html";

#ifdef HAS_QT_WEBENGINE
    auto * wordle_web_view = new WordleWebView(this);
    wordle_web_view->key_handler = [this](QKeyEvent * event) {
      return forwardWebKeyPress(event);
    };
    web_view_ = wordle_web_view;
    web_view_->setFocusPolicy(Qt::StrongFocus);
    setFocusProxy(web_view_);
    web_view_->load(QUrl::fromLocalFile(wordle_path));
    connect(web_view_, &QWebEngineView::titleChanged, this, [this](const QString & title) {
      if (!title.startsWith(QStringLiteral("feedback:"))) {
        return;
      }

      const QString feedback = title.mid(9).trimmed().toUpper();
      if (!feedback.isEmpty()) {
        emit feedbackSubmitted(feedback);
      }

      if (web_view_ != nullptr) {
        web_view_->page()->runJavaScript(
          "if (typeof window.wordleResetQtTitle === 'function') {"
          "  window.wordleResetQtTitle();"
          "}");
      }
    });
    connect(web_view_, &QWebEngineView::loadFinished, this, [this](bool ok) {
      if (!ok || web_view_ == nullptr) {
        return;
      }

      web_view_->setFocus(Qt::OtherFocusReason);
      web_view_->page()->runJavaScript("window.focus(); document.body.focus();");
    });
    QTimer::singleShot(0, web_view_, [this]() {
      if (web_view_ != nullptr) {
        web_view_->setFocus(Qt::OtherFocusReason);
      }
    });
    layout->addWidget(web_view_);
    updateScale();
    return;
#else
    status_message =
      "Wordle assets are installed at:\n" + wordle_path +
      "\n\nInstall Qt WebEngine to render the HTML game inside this panel.";
#endif
  } catch (const std::exception & ex) {
    status_message = "Failed to locate Wordle assets:\n" + QString::fromUtf8(ex.what());
  }

  auto * label = new QLabel(status_message, this);
  label->setAlignment(Qt::AlignCenter);
  label->setWordWrap(true);
  layout->addWidget(label);
}

WordleView::~WordleView()
{
  if (qApp != nullptr) {
    qApp->removeEventFilter(this);
  }
}

void WordleView::setActiveGuess(const QString & guess)
{
#ifdef HAS_QT_WEBENGINE
  if (web_view_ == nullptr) {
    return;
  }

  web_view_->page()->runJavaScript(
    QStringLiteral(
      "if (typeof window.wordleSetActiveGuess === 'function') {"
      "  window.wordleSetActiveGuess('%1');"
      "}").arg(guess.toUpper()));
#else
  (void)guess;
#endif
}

void WordleView::setDiagnosticsJson(const QString & diagnostics_json)
{
#ifdef HAS_QT_WEBENGINE
  if (web_view_ == nullptr) {
    return;
  }

  QString escaped = diagnostics_json;
  escaped.replace("\\", "\\\\");
  escaped.replace("'", "\\'");
  escaped.replace("\n", "\\n");

  web_view_->page()->runJavaScript(
    QStringLiteral(
      "if (typeof window.wordleSetDiagnostics === 'function') {"
      "  window.wordleSetDiagnostics('%1');"
      "}").arg(escaped));
#else
  (void)diagnostics_json;
#endif
}

void WordleView::previewGuess(const QString & guess)
{
#ifdef HAS_QT_WEBENGINE
  if (web_view_ == nullptr) {
    return;
  }

  web_view_->page()->runJavaScript(
    QStringLiteral(
      "if (typeof window.wordleSetQtGuess === 'function') {"
      "  window.wordleSetQtGuess('%1');"
      "}").arg(guess.toLower()));
#else
  (void)guess;
#endif
}

void WordleView::clearPreviewGuess()
{
#ifdef HAS_QT_WEBENGINE
  if (web_view_ == nullptr) {
    return;
  }

  web_view_->page()->runJavaScript(
    "if (typeof window.wordleClearQtGuess === 'function') {"
    "  window.wordleClearQtGuess();"
    "}");
#endif
}

void WordleView::submitPreviewGuess()
{
#ifdef HAS_QT_WEBENGINE
  if (web_view_ == nullptr) {
    return;
  }

  web_view_->page()->runJavaScript(
    "if (typeof window.wordleSubmitQtGuess === 'function') {"
    "  window.wordleSubmitQtGuess();"
    "}");
#endif
}

bool WordleView::eventFilter(QObject * watched, QEvent * event)
{
#ifdef HAS_QT_WEBENGINE
  (void)watched;

  if (!isVisible()) {
    return QWidget::eventFilter(watched, event);
  }

  if (event->type() == QEvent::MouseButtonPress) {
    const auto * mouse_event = static_cast<QMouseEvent *>(event);
    const QPoint local_pos = mapFromGlobal(mouse_event->globalPos());
    capture_keyboard_ = rect().contains(local_pos);

    if (capture_keyboard_ && web_view_ != nullptr) {
      web_view_->setFocus(Qt::MouseFocusReason);
    }
  }

  if (event->type() == QEvent::KeyPress && capture_keyboard_) {
    return forwardWebKeyPress(static_cast<QKeyEvent *>(event));
  }
#else
  (void)watched;
  (void)event;
#endif

  return QWidget::eventFilter(watched, event);
}

void WordleView::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updateScale();
}

void WordleView::updateScale()
{
#ifdef HAS_QT_WEBENGINE
  if (web_view_ == nullptr) {
    return;
  }

  constexpr double base_width = 460.0;
  constexpr double base_height = 700.0;

  const double width_scale = static_cast<double>(width()) / base_width;
  const double height_scale = static_cast<double>(height()) / base_height;
  const double zoom_factor = std::clamp(std::min(width_scale, height_scale), 0.7, 1.15);

  web_view_->setZoomFactor(zoom_factor);
#endif
}

#ifdef HAS_QT_WEBENGINE
bool WordleView::forwardWebKeyPress(QKeyEvent * event)
{
  if (web_view_ == nullptr) {
    return false;
  }

  const Qt::KeyboardModifiers modifiers = event->modifiers();
  if (modifiers != Qt::NoModifier && modifiers != Qt::ShiftModifier) {
    return false;
  }

  switch (event->key()) {
    case Qt::Key_Return:
    case Qt::Key_Enter:
      sendInputToPage("enter");
      return true;
    case Qt::Key_Backspace:
    case Qt::Key_Delete:
      sendInputToPage("backspace");
      return true;
    default:
      break;
  }

  const QString text = event->text().toLower();
  if (text.size() == 1 && text.front().isLetter()) {
    sendInputToPage(text);
    return true;
  }

  return false;
}

void WordleView::sendInputToPage(const QString & input)
{
  if (web_view_ == nullptr) {
    return;
  }

  web_view_->page()->runJavaScript(
    QStringLiteral(
      "if (typeof window.wordleHandleQtKey === 'function') {"
      "  window.wordleHandleQtKey('%1');"
      "}").arg(input));
}
#endif
