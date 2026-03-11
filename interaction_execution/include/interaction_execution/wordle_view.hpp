#pragma once

#include <QWidget>

class QResizeEvent;

#ifdef HAS_QT_WEBENGINE
class QWebEngineView;
#endif

class WordleView : public QWidget
{
  Q_OBJECT

public:
  explicit WordleView(QWidget * parent = nullptr);
  ~WordleView() override = default;

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  void updateScale();

#ifdef HAS_QT_WEBENGINE
  QWebEngineView * web_view_{nullptr};
#endif
};
