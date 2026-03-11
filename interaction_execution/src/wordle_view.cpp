#include "interaction_execution/wordle_view.hpp"

#include <QLabel>
#include <QResizeEvent>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>

#ifdef HAS_QT_WEBENGINE
#include <QUrl>
#include <QtWebEngineWidgets/QWebEngineView>
#endif

#include <exception>
#include <algorithm>

WordleView::WordleView(QWidget * parent)
: QWidget(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  QString status_message;

  try {
    const auto package_share_dir =
      QString::fromStdString(ament_index_cpp::get_package_share_directory("interaction_execution"));
    const auto wordle_path = package_share_dir + "/wordle-clone/index.html";

#ifdef HAS_QT_WEBENGINE
    web_view_ = new QWebEngineView(this);
    web_view_->load(QUrl::fromLocalFile(wordle_path));
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

  constexpr double base_width = 520.0;
  constexpr double base_height = 760.0;

  const double width_scale = static_cast<double>(width()) / base_width;
  const double height_scale = static_cast<double>(height()) / base_height;
  const double zoom_factor = std::clamp(std::min(width_scale, height_scale), 0.55, 1.0);

  web_view_->setZoomFactor(zoom_factor);
#endif
}
