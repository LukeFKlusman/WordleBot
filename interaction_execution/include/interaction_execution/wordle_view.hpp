#pragma once

#include <QString>
#include <QWidget>

class QEvent;
class QKeyEvent;
class QResizeEvent;

#ifdef HAS_QT_WEBENGINE
class QWebEngineView;
#endif

class WordleView : public QWidget
{
  Q_OBJECT

public:
  explicit WordleView(QWidget * parent = nullptr);
  ~WordleView() override;
  void setActiveGuess(const QString & guess);
  void setDiagnosticsJson(const QString & diagnostics_json);
  void previewGuess(const QString & guess);
  void clearPreviewGuess();
  void submitPreviewGuess();

signals:
  void feedbackSubmitted(const QString & feedback);
  void modeSelected(const QString & mode);
  void secretWordSubmitted(const QString & word);
  void playerGuessSubmitted(const QString & guess);
  void resetRequested();

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;

private:
  void updateScale();

#ifdef HAS_QT_WEBENGINE
  bool forwardWebKeyPress(QKeyEvent * event);
  void sendInputToPage(const QString & input);
  QWebEngineView * web_view_{nullptr};
  bool capture_keyboard_{true};
#endif
};
