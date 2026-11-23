#pragma once

#include <array>
#include <QImage>
#include <QJsonObject>
#include <QPointer>
#include <QStringList>
#include <QTimer>
#include <QUrl>
#include <QWidget>
#include "vision_pkg/safety_detector.hpp"

class QLabel;
class QGridLayout;
class QGroupBox;
class QNetworkAccessManager;
class QNetworkReply;
class QPushButton;
class QSpinBox;
class SafetyFrame;
class SafetyHistogram;

class SafetyTuning : public QWidget {
public:
  enum class Source { Pi, Folder };

  explicit SafetyTuning(QNetworkAccessManager &http);
  void setSource(Source source);
  void setServer(const QUrl &server);
  void setSafetyEnabled(bool enabled);
  QWidget *controlsPage() const { return controls_page_; }

private:
  void openFolder();
  void loadFolder(const QString &path);
  void browse(int step);
  void startMjpeg(const QUrl &url);
  void stopMjpeg();
  void updateFrameSource();
  void fetchFrame();
  void acceptFrame(QImage image, const QJsonObject &overlays = {});
  void syncHsv(bool save);
  void showFrame();
  vision_pkg::SafetyHsv hsv() const;

  QNetworkAccessManager &http_;
  Source source_{Source::Pi};
  QUrl server_;
  QPointer<QNetworkReply> frame_reply_, hsv_reply_, mjpeg_reply_;
  QTimer poll_, redraw_;
  QImage image_;
  QByteArray frame_tag_;
  QByteArray mjpeg_buffer_, mjpeg_frame_;
  QJsonObject overlays_;
  QRect selection_;
  QStringList files_;
  int file_index_{0};
  bool safety_enabled_{false}, legacy_video_{false};
  vision_pkg::SafetyDetector detector_;
  QGridLayout *grid_{};
  std::array<QGroupBox *, 4> groups_{};
  std::array<SafetyFrame *, 4> views_{};
  std::array<std::array<QSpinBox *, 6>, 3> inputs_{};
  SafetyHistogram *histogram_;
  QWidget *controls_page_;
  QLabel *status_;
  QLabel *frame_info_;
  QLabel *source_info_;
  QPushButton *previous_;
  QPushButton *next_;
  QPushButton *folder_;
  QPushButton *read_;
  QPushButton *apply_;
};
