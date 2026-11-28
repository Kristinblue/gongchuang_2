#include "safety_tuning.hpp"

#include <algorithm>
#include <functional>
#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QImageReader>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMouseEvent>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStandardPaths>
#include <QToolTip>
#include <QUrlQuery>
#include <QVBoxLayout>
#include <opencv2/imgproc.hpp>

namespace {
const QUrl sim_url("http://127.0.0.1:8090/");
const QUrl sim_ros_server("http://127.0.0.1:18082/");
}

class SafetyFrame : public QWidget {
public:
  std::function<void(QRect)> onSelection;
  explicit SafetyFrame() { setMinimumSize(96, 64); }
  void clearSelection() {
    selection_ = {};
    dragging_ = false;
  }
  void setImage(const QImage &image, const QString &badge = {}, const QString &stage = {}) {
    stage_ = stage;
    if (image.size() != image_.size()) clearSelection();
    image_ = image;
    badge_ = badge;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.fillRect(rect(), Qt::white);
    if (image_.isNull()) {
      painter.setPen(QColor("#748194"));
      painter.drawText(rect(), Qt::AlignCenter, "暂无画面");
      return;
    }
    const auto target = imageRect();
    painter.drawImage(target, image_);
    if (!badge_.isEmpty()) {
      const auto metrics = painter.fontMetrics();
      const auto width = std::min(target.width(), double(metrics.horizontalAdvance(badge_) + 16));
      const QRectF box(target.right() - width, target.top(), width, metrics.height() + 6);
      painter.fillRect(box, Qt::white);
      painter.setPen(QColor("#263548"));
      painter.drawText(box, Qt::AlignCenter, metrics.elidedText(badge_, Qt::ElideRight, int(width) - 12));
    }
    if (!stage_.isEmpty()) {
      auto font = painter.font();
      font.setBold(true);
      font.setPixelSize(17);
      const QFontMetrics metrics(font);
      const auto label = metrics.elidedText(stage_, Qt::ElideRight, std::max(0, int(target.width()) - 12));
      QPainterPath text;
      text.addText(target.left() + 6, target.top() + metrics.ascent() + 4, font, label);
      painter.save();
      painter.setClipRect(target);
      painter.setRenderHint(QPainter::Antialiasing);
      painter.strokePath(text, QPen(QColor(25, 25, 25), 2));
      painter.fillPath(text, QColor("#ffe04a"));
      painter.restore();
    }
    if (selection_.isEmpty()) return;
    const double scale = target.width() / image_.width();
    const QRectF box(target.x() + selection_.x() * scale, target.y() + selection_.y() * scale,
                     selection_.width() * scale, selection_.height() * scale);
    painter.setPen(QPen(QColor("#ffcc66"), 2, Qt::DashLine));
    painter.drawRect(box);
  }
  void mousePressEvent(QMouseEvent *event) override {
    if (!onSelection || image_.isNull() || !imageRect().contains(event->position())) return;
    if (event->button() == Qt::RightButton) {
      selection_ = {};
      onSelection(selection_);
      update();
    } else if (event->button() == Qt::LeftButton) {
      start_ = imagePoint(event->position());
      dragging_ = true;
    }
  }
  void mouseMoveEvent(QMouseEvent *event) override {
    if (!dragging_ || image_.isNull()) return;
    selection_ = QRect(start_, imagePoint(event->position())).normalized();
    update();
  }
  void mouseReleaseEvent(QMouseEvent *event) override {
    if (!dragging_) return;
    dragging_ = false;
    if (image_.isNull()) return;
    selection_ = QRect(start_, imagePoint(event->position())).normalized();
    onSelection(selection_);
    update();
  }

private:
  QRectF imageRect() const {
    const QSize size = image_.size().scaled(this->size(), Qt::KeepAspectRatio);
    return QRectF((width() - size.width()) / 2., (height() - size.height()) / 2., size.width(), size.height());
  }
  QPoint imagePoint(const QPointF &p) const {
    const auto target = imageRect();
    return {std::clamp(int((p.x() - target.x()) * image_.width() / target.width()), 0, image_.width() - 1),
            std::clamp(int((p.y() - target.y()) * image_.height() / target.height()), 0, image_.height() - 1)};
  }
  QImage image_;
  QString badge_, stage_;
  QPoint start_;
  QRect selection_;
  bool dragging_{false};
};

class SafetyHistogram : public QWidget {
public:
  SafetyHistogram() {
    setMinimumHeight(183);
    setMouseTracking(true);
  }
  void setHsv(const cv::Mat &hsv) {
    for (int i = 0; i < 3; ++i) {
      if (hsv.empty()) {
        bins_[i].release();
        continue;
      }
      const int size = i ? 256 : 180;
      const float range[]{0, static_cast<float>(size)};
      const float *ranges[]{range};
      cv::calcHist(&hsv, 1, &i, {}, bins_[i], 1, &size, ranges);
    }
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const QColor colors[]{QColor("#8052b9"), QColor("#087969"), QColor("#ab611c")};
    for (int i = 0; i < 3; ++i) {
      const auto plot = plotRect(i);
      painter.setPen(QColor("#aab5c0"));
      painter.drawLine(plot.bottomLeft(), plot.bottomRight());
      painter.setPen(colors[i]);
      painter.drawText(QPointF(6, plot.center().y()), QString("HSV").mid(i, 1));
      painter.drawText(plot.bottomLeft() + QPointF(0, 13), "0");
      painter.drawText(plot.bottomRight() + QPointF(-20, 13), i ? "255" : "179");
      if (bins_[i].empty()) continue;
      double peak = 0;
      cv::minMaxLoc(bins_[i], nullptr, &peak);
      if (!peak) continue;
      QPainterPath path;
      for (int j = 0; j < bins_[i].rows; ++j) {
        const QPointF p(plot.left() + j * plot.width() / (bins_[i].rows - 1),
                        plot.bottom() - plot.height() * bins_[i].at<float>(j) / peak);
        if (j) path.lineTo(p);
        else path.moveTo(p);
      }
      painter.drawPath(path);
    }
  }

  void mouseMoveEvent(QMouseEvent *event) override {
    for (int i = 0; i < 3; ++i) {
      const auto plot = plotRect(i);
      if (!plot.contains(event->position()) || bins_[i].empty()) continue;
      const int bin = qRound((event->position().x() - plot.x()) / plot.width() * (bins_[i].rows - 1));
      QToolTip::showText(event->globalPosition().toPoint(),
        QString("%1 %2 · %3 像素").arg(QString("HSV").mid(i, 1)).arg(bin).arg(bins_[i].at<float>(bin), 0, 'f', 0), this);
      return;
    }
    QToolTip::hideText();
  }

private:
  QRectF plotRect(int channel) const {
    const double height = this->height() / 3.;
    return QRectF(28, channel * height + 4, width() - 42, height - 20);
  }
  std::array<cv::Mat, 3> bins_;
};

SafetyTuning::SafetyTuning(QNetworkAccessManager &http) : http_(http) {
  QSettings saved;
  auto *page = new QVBoxLayout(this);
  auto *source_bar = new QHBoxLayout;
  folder_ = new QPushButton("打开图片文件夹");
  sim_start_ = new QPushButton("启动仿真");
  sim_task_start_ = new QPushButton("运行仿真任务");
  sim_task_reset_ = new QPushButton("复位");
  sim_task_start_->setEnabled(false);
  sim_task_reset_->setEnabled(false);
  sim_overview_ = new QPushButton("3D 全景");
  sim_config_ = new QPushButton("环境配置");
  previous_ = new QPushButton("← A");
  next_ = new QPushButton("D →");
  source_info_ = new QLabel;
  source_info_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  frame_info_ = new QLabel("等待原图");
  source_bar->addWidget(folder_);
  source_bar->addWidget(sim_start_);
  source_bar->addWidget(sim_task_start_);
  source_bar->addWidget(sim_task_reset_);
  source_bar->addWidget(sim_overview_);
  source_bar->addWidget(sim_config_);
  source_bar->addWidget(source_info_, 1);
  source_bar->addWidget(frame_info_);
  source_bar->addWidget(previous_);
  source_bar->addWidget(next_);
  page->addLayout(source_bar);
  auto *layout = new QHBoxLayout;
  page->addLayout(layout, 1);
  grid_ = new QGridLayout;
  const QString titles[]{"原图 · 拖框取样", "紫色掩膜", "红蓝掩膜", "识别结果"};
  for (int i = 0; i < 4; ++i) {
    auto *group = groups_[i] = new QGroupBox(titles[i]);
    auto *box = new QVBoxLayout(group);
    views_[i] = new SafetyFrame;
    box->addWidget(views_[i]);
    grid_->addWidget(group, i / 2, i % 2);
  }
  views_[0]->setFocusPolicy(Qt::ClickFocus);
  layout->addLayout(grid_, 1);
  auto *scroll = new QScrollArea(this);
  controls_page_ = scroll;
  scroll->setObjectName("hsvControls");
  scroll->setWidgetResizable(true);
  auto *side = new QWidget;
  auto *controls = new QVBoxLayout(side);
  const QString color_titles[]{"紫色", "红色", "蓝色"};
  for (std::size_t color = 0; color < inputs_.size(); ++color) {
    auto *group = new QGroupBox(color_titles[color]);
    auto *fields = new QGridLayout(group);
    const auto saved_values = saved.value(QString("hsv/%1").arg(vision_pkg::hsv_names[color])).toList();
    for (int j = 0; j < 3; ++j) fields->addWidget(new QLabel(QString("HSV").mid(j, 1)), 0, j + 1);
    for (int row = 0; row < 2; ++row) fields->addWidget(new QLabel(row ? "最大" : "最小"), row + 1, 0);
    for (std::size_t i = 0; i < inputs_[color].size(); ++i) {
      auto *spin = inputs_[color][i] = new QSpinBox;
      spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
      spin->setAlignment(Qt::AlignCenter);
      spin->setRange(0, i % 3 == 0 ? 179 : 255);
      spin->setValue(saved_values.size() == 6 ? saved_values.at(i).toInt() : vision_pkg::default_hsv[color][i]);
      spin->setKeyboardTracking(false);
      fields->addWidget(spin, i / 3 + 1, i % 3 + 1);
      connect(spin, &QSpinBox::valueChanged, this, [this] {
        status_->setText(server_.isEmpty() ? "已保存本地阈值" : "已保存本地阈值，可应用到香橙派");
        redraw_.start();
      });
    }
    controls->addWidget(group);
  }
  auto *buttons = new QHBoxLayout;
  read_ = new QPushButton("读取香橙派");
  apply_ = new QPushButton("应用到香橙派");
  buttons->addWidget(read_);
  buttons->addWidget(apply_);
  controls->addLayout(buttons);
  status_ = new QLabel("本地 HSV 阈值");
  status_->setWordWrap(true);
  controls->addWidget(status_);
  auto *histogram = new QGroupBox("HSV 直方图 · 右键取消选区");
  auto *hist_layout = new QVBoxLayout(histogram);
  histogram_ = new SafetyHistogram;
  hist_layout->addWidget(histogram_);
  controls->addWidget(histogram);
  controls->addStretch(1);
  scroll->setWidget(side);
  read_->setEnabled(false);
  apply_->setEnabled(false);
  connect(read_, &QPushButton::clicked, this, [this] { syncHsv(false); });
  connect(apply_, &QPushButton::clicked, this, [this] { syncHsv(true); });
  connect(&poll_, &QTimer::timeout, this, [this] {
    fetchFrame();
    if (!mjpeg_frame_.isEmpty()) {
      QImage frame = QImage::fromData(mjpeg_frame_, "JPEG");
      mjpeg_frame_.clear();
      if (!frame.isNull()) {
        source_info_->setText(source_ == Source::Pi ? "香橙派画面" : "仿真车载画面");
        if (source_ == Source::Sim) {
          sim_start_->setEnabled(false);
          const bool ready = !sim_resetting_ && sim_.state() == QProcess::Running &&
                             sim_task_.state() == QProcess::NotRunning;
          sim_task_start_->setEnabled(ready);
          sim_task_reset_->setEnabled(ready);
        }
        acceptFrame(frame);
      }
    }
  });
  poll_.setInterval(180);
  redraw_.setInterval(25);
  redraw_.setSingleShot(true);
  connect(&redraw_, &QTimer::timeout, this, [this] {
    QSettings saved;
    const auto ranges = hsv();
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      if (!vision_pkg::valid_hsv(ranges[i])) continue;
      QVariantList values;
      for (int value : ranges[i]) values << value;
      saved.setValue(QString("hsv/%1").arg(vision_pkg::hsv_names[i]), values);
    }
    showFrame();
  });
  connect(folder_, &QPushButton::clicked, this, [this] { openFolder(); });
  connect(sim_start_, &QPushButton::clicked, this, [this] { startSim(); });
  connect(sim_task_start_, &QPushButton::clicked, this, [this] { runSimTask("--start"); });
  connect(sim_task_reset_, &QPushButton::clicked, this, [this] { resetSim(); });
  sim_task_.setProcessChannelMode(QProcess::MergedChannels);
  connect(&sim_task_, &QProcess::finished, this, [this](int code, QProcess::ExitStatus) {
    const auto output = QString::fromUtf8(sim_task_.readAll()).trimmed();
    if (source_ != Source::Sim) return;
    sim_task_reset_->setEnabled(sim_.state() == QProcess::Running);
    source_info_->setToolTip(output);
    if (code || !output.contains("success=True"))
      source_info_->setText("仿真任务请求失败");
  });
  connect(sim_overview_, &QPushButton::clicked, this, [this] { openOverview(); });
  connect(sim_config_, &QPushButton::clicked, this, [this] { showSimConfig(); });
  sim_.setProcessChannelMode(QProcess::MergedChannels);
  connect(&sim_, &QProcess::readyRead, this, [this] {
    sim_error_ = (sim_error_ + QString::fromUtf8(sim_.readAll())).right(4096);
  });
  connect(&sim_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error != QProcess::FailedToStart) return;
    sim_start_->setEnabled(true);
    source_info_->setToolTip(sim_.errorString());
    source_info_->setText("仿真启动失败");
  });
  connect(&sim_, &QProcess::finished, this, [this](int code, QProcess::ExitStatus status) {
    if (sim_resetting_) {
      sim_resetting_ = false;
      if (code == 0 && status == QProcess::NormalExit) {
        startSim();
        return;
      }
    }
    sim_start_->setEnabled(true);
    sim_task_start_->setEnabled(false);
    sim_task_reset_->setEnabled(false);
    if (source_ != Source::Sim) return;
    source_info_->setToolTip(sim_error_);
    source_info_->setText(code == 0 ? "仿真已退出" : "仿真退出，悬停查看日志");
  });
  connect(previous_, &QPushButton::clicked, this, [this] { browse(-1); });
  connect(next_, &QPushButton::clicked, this, [this] { browse(1); });
  for (auto *scope : {static_cast<QWidget *>(this), controls_page_}) {
    auto *previous_key = new QShortcut(QKeySequence(Qt::Key_A), scope);
    auto *next_key = new QShortcut(QKeySequence(Qt::Key_D), scope);
    previous_key->setContext(Qt::WidgetWithChildrenShortcut);
    next_key->setContext(Qt::WidgetWithChildrenShortcut);
    connect(previous_key, &QShortcut::activated, this, [this] { browse(-1); });
    connect(next_key, &QShortcut::activated, this, [this] { browse(1); });
  }
  setSafetyEnabled(false);
  setSource(Source::Pi);
}

void SafetyTuning::setSafetyEnabled(bool enabled) {
  safety_enabled_ = enabled;
  grid_->removeWidget(groups_[0]);
  grid_->addWidget(groups_[0], 0, 0, 1, enabled ? 1 : 2);
  groups_[0]->setTitle(enabled ? "原图 · 拖框取样" : "画面");
  for (std::size_t i = 1; i < groups_.size(); ++i) groups_[i]->setVisible(enabled);
  views_[0]->clearSelection();
  selection_ = {};
  views_[0]->onSelection = {};
  if (enabled) {
    views_[0]->onSelection = [this](QRect area) {
      selection_ = area;
      showFrame();
    };
    if (source_ == Source::Pi) syncHsv(false);
  }
  showFrame();
}

void SafetyTuning::openFolder() {
  const auto start = QSettings().value("input/folder").toString();
  const auto path = QFileDialog::getExistingDirectory(this, "选择图片文件夹", start);
  if (!path.isEmpty()) loadFolder(path);
}

void SafetyTuning::loadFolder(const QString &path) {
  if (path.isEmpty()) return;
  const auto formats = QImageReader::supportedImageFormats();
  QStringList files;
  for (const auto &entry : QDir(path).entryInfoList(QDir::Files | QDir::Readable, QDir::Name | QDir::IgnoreCase))
    if (formats.contains(entry.suffix().toLatin1().toLower())) files.append(entry.absoluteFilePath());
  if (files.isEmpty()) {
    source_info_->setText("文件夹里没有图片");
    return;
  }
  files_ = files;
  file_index_ = 0;
  QSettings().setValue("input/folder", path);
  updateFrameSource();
  browse(0);
}

void SafetyTuning::browse(int step) {
  if (source_ != Source::Folder || files_.isEmpty()) return;
  const auto count = files_.size();
  file_index_ = (file_index_ + step + count) % count;
  const auto &path = files_.at(file_index_);
  QImageReader reader(path);
  reader.setAutoTransform(true);
  const auto image = reader.read();
  selection_ = {};
  views_[0]->clearSelection();
  const auto name = QFileInfo(path).fileName();
  source_info_->setText(QString("%1 / %2 · %3%4").arg(file_index_ + 1).arg(count)
    .arg(name, image.isNull() ? " · 读取失败" : ""));
  source_info_->setToolTip(path);
  acceptFrame(image);
}

void SafetyTuning::setSource(Source source) {
  source_ = source;
  poll_.stop();
  auto *reply = frame_reply_.data();
  frame_reply_.clear();
  if (reply) reply->abort();
  stopMjpeg();
  sim_task_start_->setEnabled(false);
  sim_task_reset_->setEnabled(source == Source::Sim && !sim_resetting_ &&
                             sim_.state() == QProcess::Running);
  files_.clear();
  frame_tag_.clear();
  legacy_video_ = false;
  source_info_->setToolTip({});
  folder_->setVisible(source == Source::Folder);
  previous_->setVisible(source == Source::Folder);
  next_->setVisible(source == Source::Folder);
  for (auto *button : {sim_start_, sim_task_start_, sim_task_reset_, sim_overview_, sim_config_})
    button->setVisible(source == Source::Sim);
  acceptFrame({});
  updateFrameSource();
  if (source == Source::Folder) {
    source_info_->setText("请选择图片文件夹");
    loadFolder(QSettings().value("input/folder").toString());
  } else if (source == Source::Sim) {
    source_info_->setText(sim_resetting_ ? "正在复位场景" : "等待仿真画面");
    if (!sim_resetting_) {
      if (sim_.state() == QProcess::NotRunning) startMjpeg(sim_url.resolved(QUrl("video")));
      else poll_.start();
    }
  } else {
    source_info_->setText(server_.isEmpty() ? "等待香橙派连接" : "等待香橙派画面");
  }
}

SafetyTuning::~SafetyTuning() {
  sim_resetting_ = false;
  if (sim_.state() != QProcess::NotRunning) {
    sim_.closeWriteChannel();
    sim_.waitForFinished(12000);
  }
}

void SafetyTuning::startSim() {
  QDir dir(QCoreApplication::applicationDirPath());
  while (!dir.exists("tools/sim/stream.py") && dir.cdUp()) {}
  auto python = qEnvironmentVariable("GC_SIM_PY");
  if (python.isEmpty()) {
    // 与原电脑仿真入口共用已安装依赖的 Python。
    python = "D:/MySoftware/miniconda3/envs/yolov8/python.exe";
    if (!QFileInfo::exists(python)) python = QStandardPaths::findExecutable("python");
  }
  if (!dir.exists("tools/sim/stream.py") || python.isEmpty()) {
    source_info_->setText("启动仿真需要工程目录和 Python");
    return;
  }
  if (sim_.state() != QProcess::NotRunning) return;
  sim_error_.clear();
  sim_start_->setEnabled(false);
  sim_.setWorkingDirectory(dir.absolutePath());
  sim_.start(python, {"-X", "utf8", dir.absoluteFilePath("tools/sim/stream.py"), "--client", "--ros"});
  if (source_ == Source::Sim) {
    source_info_->setToolTip({});
    source_info_->setText("正在启动仿真");
    stopMjpeg();
    frame_tag_.clear();
    poll_.start();
  }
}

void SafetyTuning::resetSim() {
  if (sim_resetting_ || sim_.state() != QProcess::Running || sim_task_.state() != QProcess::NotRunning)
    return;
  sim_resetting_ = true;
  sim_task_start_->setEnabled(false);
  sim_task_reset_->setEnabled(false);
  auto *reply = frame_reply_.data();
  frame_reply_.clear();
  if (reply) reply->abort();
  stopMjpeg();
  acceptFrame({});
  source_info_->setText("正在复位场景");
  // 先让旧场景和 ROS 正常退出，finished 回调再加载初始场景。
  sim_.closeWriteChannel();
}

void SafetyTuning::openOverview() {
  QNetworkRequest request(sim_url.resolved(QUrl("overview")));
  request.setTransferTimeout(2500);
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  auto *reply = http_.post(request, QByteArray());
  connect(reply, &QNetworkReply::finished, this, [this, reply] {
    if (reply->error() != QNetworkReply::NoError && source_ == Source::Sim)
      source_info_->setText("全景尚未就绪");
    reply->deleteLater();
  });
}

void SafetyTuning::runSimTask(const QString &action) {
  if (sim_.state() != QProcess::Running || sim_task_.state() != QProcess::NotRunning) return;
  sim_task_start_->setEnabled(false);
  sim_task_reset_->setEnabled(false);
  sim_task_.setWorkingDirectory(sim_.workingDirectory());
  sim_task_.start("wsl.exe", {"-d", "Ubuntu-22.04", "--", "bash", "tools/sim/run_sim.sh", action});
}

void SafetyTuning::showSimConfig() {
  QNetworkRequest request(sim_url.resolved(QUrl("config")));
  request.setTransferTimeout(2500);
  auto *reply = http_.get(request);
  connect(reply, &QNetworkReply::finished, this, [this, reply] {
    const auto config = QJsonDocument::fromJson(reply->readAll());
    reply->deleteLater();
    if (source_ != Source::Sim) return;
    if (!config.isObject()) {
      source_info_->setText("请先启动仿真，再查看环境配置");
      return;
    }
    auto *dialog = new QDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle("仿真环境配置");
    auto *layout = new QVBoxLayout(dialog);
    auto *text = new QPlainTextEdit(QString::fromUtf8(config.toJson()));
    text->setReadOnly(true);
    layout->addWidget(text);
    dialog->resize(720, 560);
    dialog->show();
  });
}

void SafetyTuning::stopMjpeg() {
  auto *prior = mjpeg_reply_.data();
  mjpeg_reply_.clear();
  if (prior) prior->abort();
  mjpeg_buffer_.clear();
  mjpeg_frame_.clear();
}

void SafetyTuning::startMjpeg(const QUrl &url) {
  stopMjpeg();
  QNetworkRequest request(url);
  request.setTransferTimeout(5000);
  auto *reply = http_.get(request);
  mjpeg_reply_ = reply;
  connect(reply, &QNetworkReply::readyRead, this, [this, reply] {
    if (mjpeg_reply_ != reply) return;
    mjpeg_buffer_ += reply->readAll();
    const QByteArray soi("\xff\xd8", 2), eoi("\xff\xd9", 2);
    while (true) {
      const auto start = mjpeg_buffer_.indexOf(soi);
      if (start < 0) {
        mjpeg_buffer_ = mjpeg_buffer_.right(1);
        break;
      }
      if (start) mjpeg_buffer_.remove(0, start);
      const auto end = mjpeg_buffer_.indexOf(eoi, 2);
      if (end < 0) break;
      mjpeg_frame_ = mjpeg_buffer_.left(end + 2);
      mjpeg_buffer_.remove(0, end + 2);
    }
    if (mjpeg_buffer_.size() > 8 * 1024 * 1024) mjpeg_buffer_.clear();
  });
  connect(reply, &QNetworkReply::finished, this, [this, reply, url] {
    reply->deleteLater();
    if (mjpeg_reply_ != reply) return;
    mjpeg_reply_.clear();
    if (source_ == Source::Pi) {
      source_info_->setText("等待香橙派画面");
      QTimer::singleShot(1000, this, [this, url] {
        if (source_ == Source::Pi && legacy_video_ && !mjpeg_reply_ && server_.resolved(QUrl("/video")) == url)
          startMjpeg(url);
      });
      return;
    }
    acceptFrame({});
    sim_task_start_->setEnabled(false);
    if (sim_.state() == QProcess::NotRunning) {
      sim_start_->setEnabled(true);
      if (sim_error_.isEmpty()) source_info_->setText("等待仿真，可点击启动仿真");
    }
    QTimer::singleShot(1000, this, [this] {
      if (source_ == Source::Sim && !sim_resetting_ && !mjpeg_reply_)
        startMjpeg(sim_url.resolved(QUrl("video")));
    });
  });
  poll_.start();
}

void SafetyTuning::updateFrameSource() {
  previous_->setEnabled(files_.size() > 1);
  next_->setEnabled(files_.size() > 1);
  apply_->setEnabled(!server_.isEmpty() && !hsv_reply_);
  if (source_ == Source::Pi && !server_.isEmpty()) {
    poll_.start();
    fetchFrame();
  }
}

void SafetyTuning::setServer(const QUrl &server) {
  if (server_ == server) return;
  server_ = server;
  frame_tag_.clear();
  for (auto *slot : {&frame_reply_, &hsv_reply_}) {
    auto *reply = slot->data();
    slot->clear();
    if (reply) reply->abort();
  }
  if (source_ == Source::Pi) {
    poll_.stop();
    stopMjpeg();
    legacy_video_ = false;
    acceptFrame({});
    source_info_->setText(server_.isEmpty() ? "等待香橙派连接" : "等待香橙派画面");
  }
  read_->setEnabled(!server_.isEmpty());
  for (auto &group : inputs_)
    for (auto *input : group) input->setEnabled(true);
  status_->setText(server_.isEmpty() ? "本地 HSV 阈值" : "本地阈值可读取或应用到香橙派");
  if (!server_.isEmpty() && source_ == Source::Pi) syncHsv(false);
  updateFrameSource();
}

void SafetyTuning::fetchFrame() {
  const bool sim = source_ == Source::Sim;
  if (frame_reply_ || legacy_video_) return;
  if (sim ? (sim_resetting_ || sim_.state() != QProcess::Running)
          : (source_ != Source::Pi || server_.isEmpty())) return;
  const auto server = sim ? sim_ros_server : server_;
  QNetworkRequest request(server.resolved(QUrl("/safety/frame?format=jpeg")));
  request.setTransferTimeout(3000);
  if (!frame_tag_.isEmpty()) request.setRawHeader("If-None-Match", frame_tag_);
  auto *reply = http_.get(request);
  frame_reply_ = reply;
  connect(reply, &QNetworkReply::finished, this, [this, reply, sim, server] {
    reply->deleteLater();
    if (frame_reply_ != reply) return;
    frame_reply_.clear();
    const auto overlays = QJsonDocument::fromJson(reply->rawHeader("X-Gongchuang-Vision")).object();
    const auto code = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    if (code == 404) {
      legacy_video_ = true;
      startMjpeg(server.resolved(QUrl("/video")));
      return;
    }
    if (code == 304) {
      if (overlays_ != overlays) {
        overlays_ = overlays;
        showFrame();
      }
      return;
    }
    QImage next;
    if (reply->error() != QNetworkReply::NoError || !next.loadFromData(reply->readAll())) {
      frame_tag_.clear();
      source_info_->setText(sim ? "等待仿真画面" : "等待香橙派画面");
    } else {
      frame_tag_ = reply->rawHeader("ETag");
      source_info_->setText(sim ? "仿真车载画面" : "香橙派画面");
      if (sim) {
        const bool ready = !sim_resetting_ && sim_task_.state() == QProcess::NotRunning;
        sim_task_start_->setEnabled(ready);
        sim_task_reset_->setEnabled(ready);
      }
    }
    acceptFrame(next, next.isNull() ? QJsonObject{} : overlays);
  });
}

void SafetyTuning::acceptFrame(QImage image, const QJsonObject &overlays) {
  if (image_.size() != image.size()) selection_ = {};
  image_ = image.convertToFormat(QImage::Format_BGR888);
  overlays_ = overlays;
  showFrame();
}

vision_pkg::SafetyHsv SafetyTuning::hsv() const {
  vision_pkg::SafetyHsv result{};
  for (std::size_t i = 0; i < result.size(); ++i)
    for (std::size_t j = 0; j < result[i].size(); ++j) result[i][j] = inputs_[i][j]->value();
  return result;
}

void SafetyTuning::syncHsv(bool save) {
  if (!safety_enabled_ || server_.isEmpty() || hsv_reply_) return;
  QNetworkRequest request(server_.resolved(QUrl("/safety/hsv")));
  request.setTransferTimeout(3000);
  if (save) {
    const auto ranges = hsv();
    QUrlQuery fields;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      if (!vision_pkg::valid_hsv(ranges[i])) {
        status_->setText("S、V 的最小值须小于或等于最大值");
        return;
      }
      QStringList values;
      for (const auto value : ranges[i]) values << QString::number(value);
      fields.addQueryItem(vision_pkg::hsv_names[i], values.join(','));
    }
    request.setRawHeader("X-Gongchuang-Client", "qt");
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");
    hsv_reply_ = http_.post(request, fields.toString(QUrl::FullyEncoded).toUtf8());
  } else {
    hsv_reply_ = http_.get(request);
  }
  auto *reply = hsv_reply_.data();
  read_->setEnabled(false);
  apply_->setEnabled(false);
  for (auto &group : inputs_)
    for (auto *input : group) input->setEnabled(false);
  connect(reply, &QNetworkReply::finished, this, [this, reply, save] {
    reply->deleteLater();
    if (hsv_reply_ != reply) return;
    hsv_reply_.clear();
    const auto object = QJsonDocument::fromJson(reply->readAll()).object();
    bool synced = reply->error() == QNetworkReply::NoError;
    for (std::size_t i = 0; synced && i < inputs_.size(); ++i)
      synced = object.value(vision_pkg::hsv_names[i]).toArray().size() == 6;
    if (synced) {
      for (std::size_t i = 0; i < inputs_.size(); ++i) {
        const auto values = object.value(vision_pkg::hsv_names[i]).toArray();
        for (std::size_t j = 0; j < inputs_[i].size(); ++j) {
          QSignalBlocker block(inputs_[i][j]);
          inputs_[i][j]->setValue(values.at(j).toInt());
        }
      }
      status_->setText(save ? "香橙派阈值已生效" : "已读取香橙派阈值");
    } else {
      status_->setText(save ? "应用失败，重新读取香橙派阈值后可重试" : "无法读取，请确认安全区节点已启动");
    }
    for (auto &group : inputs_)
      for (auto *input : group) input->setEnabled(true);
    read_->setEnabled(true);
    apply_->setEnabled(true);
    showFrame();
  });
}

void SafetyTuning::showFrame() {
  if (image_.isNull()) {
    for (auto *view : views_) view->setImage({});
    histogram_->setHsv({});
    frame_info_->setText("等待原图");
    return;
  }
  const cv::Mat bgr(image_.height(), image_.width(), CV_8UC3, image_.bits(), image_.bytesPerLine());
  const int thickness = std::max(2, bgr.cols / 480);
  const double font_size = std::max(.45, bgr.cols / 1800.);
  const auto draw_box = [&](cv::Mat &canvas, const QJsonArray &box, bool target) {
    if (box.size() != 5) return;
    const cv::Point upper(qRound(std::clamp(box[1].toDouble(), 0., double(canvas.cols - 1))),
                          qRound(std::clamp(box[2].toDouble(), 0., double(canvas.rows - 1))));
    const cv::Point lower(qRound(std::clamp(box[3].toDouble(), 0., double(canvas.cols - 1))),
                          qRound(std::clamp(box[4].toDouble(), 0., double(canvas.rows - 1))));
    if (lower.x <= upper.x || lower.y <= upper.y) return;
    const auto name = box[0].toString();
    cv::Scalar color(155, 120, 115);
    if (name == "green") color = {40, 185, 35};
    else if (name == "orange") color = {0, 125, 255};
    else if (name == "blue") color = {255, 135, 35};
    else if (name == "black") color = {25, 25, 25};
    if (target) color = {0, 220, 255};
    cv::rectangle(canvas, upper, lower, color, thickness);
    cv::putText(canvas, target ? ("target " + name).toStdString() : name.toStdString(),
      {upper.x, std::max(18, upper.y - 4)}, cv::FONT_HERSHEY_SIMPLEX, font_size, color, thickness, cv::LINE_AA);
  };
  const auto boxes = overlays_.value("boxes").toArray();
  const auto draw_reference = [&](cv::Mat &canvas) {
    draw_box(canvas, overlays_.value("target").toArray(), true);
    const auto point = overlays_.value("point").toArray();
    if (point.size() != 2) return;
    const cv::Point pixel(qRound(point[0].toDouble()), qRound(point[1].toDouble()));
    if (cv::Rect(0, 0, canvas.cols, canvas.rows).contains(pixel))
      cv::drawMarker(canvas, pixel, {70, 180, 0}, cv::MARKER_CROSS, 16, thickness, cv::LINE_AA);
  };
  QImage preview = image_;
  if (!boxes.isEmpty() || (!safety_enabled_ && !overlays_.isEmpty())) {
    cv::Mat marked(preview.height(), preview.width(), CV_8UC3, preview.bits(), preview.bytesPerLine());
    for (const auto &box : boxes) draw_box(marked, box.toArray(), false);
    if (!safety_enabled_) draw_reference(marked);
  }
  const auto stage = overlays_.value("stage").toString();
  const auto title = stage.isEmpty() ? QString("waiting") : QString(stage).replace("/", " / ");
  views_[0]->setImage(preview, {}, title);
  frame_info_->setText(QString("%1 × %2").arg(image_.width()).arg(image_.height()));
  if (!safety_enabled_) return;
  const QRect sample = selection_.isEmpty() ? image_.rect() : selection_.intersected(image_.rect());
  cv::Mat hsv_image;
  cv::cvtColor(bgr(cv::Rect(sample.x(), sample.y(), sample.width(), sample.height())), hsv_image, cv::COLOR_BGR2HSV);
  histogram_->setHsv(hsv_image);
  const auto ranges = hsv();
  if (!std::all_of(ranges.begin(), ranges.end(), vision_pkg::valid_hsv)) {
    for (std::size_t i = 1; i < views_.size(); ++i) views_[i]->setImage({});
    return;
  }

  vision_pkg::SafetyDebug debug;
  const auto result = detector_.detect(bgr, ranges, &debug);
  cv::Mat purple = cv::Mat::zeros(bgr.size(), CV_8UC1);
  cv::Mat colors = cv::Mat::zeros(bgr.size(), CV_8UC3);
  cv::Mat overlay = bgr.clone();
  for (const auto &box : boxes) draw_box(overlay, box.toArray(), false);
  if (!debug.purple.empty()) debug.purple.copyTo(purple(debug.roi));
  if (!debug.red.empty()) {
    cv::insertChannel(debug.red, colors(debug.roi), 2);
    cv::insertChannel(debug.blue, colors(debug.roi), 0);
  }
  if (result.status.detected) {
    cv::polylines(overlay, result.polygon, true, result.status.full ? cv::Scalar(0, 220, 255) : cv::Scalar(0, 130, 255), 2, cv::LINE_AA);
  }
  draw_reference(overlay);
  views_[1]->setImage(QImage(purple.data, purple.cols, purple.rows, purple.step, QImage::Format_Grayscale8).copy());
  views_[2]->setImage(QImage(colors.data, colors.cols, colors.rows, colors.step, QImage::Format_BGR888).copy());
  views_[3]->setImage(QImage(overlay.data, overlay.cols, overlay.rows, overlay.step, QImage::Format_BGR888).copy(),
    QString::fromStdString(result.flag) + (result.status.ok ? " · ok=True" : " · ok=False"), title);
}
