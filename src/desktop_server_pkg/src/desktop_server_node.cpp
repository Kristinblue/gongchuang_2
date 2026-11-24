#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <future>
#include <optional>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <robot_interfaces/msg/match_state.hpp>
#include <robot_interfaces/msg/feedback.hpp>
#include <robot_interfaces/msg/serial_frame.hpp>
#include <robot_interfaces/msg/state.hpp>
#include <robot_interfaces/msg/objects.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include "httplib.h"
#include "safety_http.hpp"

using namespace std::chrono_literals;
using Image = sensor_msgs::msg::Image;
using Objects = robot_interfaces::msg::Objects;
using RobotState = robot_interfaces::msg::State;
using Feedback = robot_interfaces::msg::Feedback;
using SerialFrame = robot_interfaces::msg::SerialFrame;
using MatchState = robot_interfaces::msg::MatchState;
using ReadSettings = rcl_interfaces::srv::GetParameters;
using WriteSettings = rcl_interfaces::srv::SetParametersAtomically;

// 香橙派上的预览服务：读 ROS 画面和状态，给电脑提供 JPEG 视频、状态和设置。
class DesktopServer : public rclcpp::Node {
public:
  DesktopServer() : Node("desktop_server_node") {
    const auto host = declare_parameter("host", "127.0.0.1");
    const int port = declare_parameter("port", 8080);
    const auto source = declare_parameter("camera_source", "usb");
    const auto mode = declare_parameter("mode", "preview");
    // 靠近测试用的参考点，画在预览画面上，坐标以相机原图为准。
    const auto point_x = declare_parameter("point.x0", 640.0);
    const auto point_y = declare_parameter("point.y0", 550.0);
    if (source != "usb" && source != "http") throw std::invalid_argument("camera_source");
    const auto width = declare_parameter("preview_width", 640);
    const auto height = declare_parameter("preview_height", 360);
    quality_ = declare_parameter("jpeg_quality", 65);
    if (port < 1 || port > 65535 || width < 1 || height < 1 || quality_ < 25 || quality_ > 90)
      throw std::invalid_argument("HTTP port, preview dimensions or JPEG quality");
    // 置信度属于推理节点。HTTP 收到设置后，通过 ROS 参数服务转给推理节点。
    read_settings_ = create_client<ReadSettings>("inference_node/get_parameters");
    write_settings_ = create_client<WriteSettings>("inference_node/set_parameters_atomically");
    // 这几个 ROS callback 只存最新值，绘图和压缩交给下面的 encode_loop。
    robot_ = create_subscription<RobotState>("robot/state", rclcpp::QoS(1).transient_local(),
      [this](const RobotState &state) {
        std::lock_guard lock(mutex_);
        serial_state_ = state;
        if (!state.open) serial_feedback_.reset();
        camera_angle_ = state.camera_known ? std::optional<int>{state.camera_deg} : std::nullopt;
      });
    // 只订阅已经解析好的回包，串口仍由通信节点独占。
    feedback_ = create_subscription<Feedback>("robot/feedback", 10, [this](const Feedback &message) {
      std::lock_guard lock(mutex_);
      serial_feedback_ = message;
    });
    serial_frames_ = create_subscription<SerialFrame>("robot/serial_frames", 100, [this](const SerialFrame &frame) {
      constexpr char hex[] = "0123456789ABCDEF";
      std::string line = frame.tx ? "TX" : "RX";
      for (auto byte : frame.data) {
        line += ' ';
        line += hex[byte >> 4];
        line += hex[byte & 15];
      }
      std::lock_guard lock(mutex_);
      ++serial_seq_;
      if (serial_lines_.size() == 100) serial_lines_.pop_front();
      serial_lines_.push_back(std::move(line));
    });
    target_ = create_subscription<Objects>("task/target", rclcpp::SensorDataQoS().keep_last(1),
      [this](Objects::ConstSharedPtr message) {
        std::lock_guard lock(mutex_);
        selected_ = std::move(message);
      });
    status_ = create_subscription<MatchState>("match/state", rclcpp::QoS(1).transient_local(),
      [this](const MatchState &message) {
        std::lock_guard lock(mutex_);
        stage_ = message.stage;
        frames_ = message.frames;
        if (stage_ != "track" && stage_ != "grab") selected_.reset();
      });
    input_ = create_subscription<Image>("vision/preview", rclcpp::SensorDataQoS().keep_last(1),
      [this](Image::ConstSharedPtr image) {
        std::lock_guard lock(mutex_);
        newest_ = std::move(image);
        frame_received_ = std::chrono::steady_clock::now();
      });

    server_.set_socket_options([](socket_t socket) {
      int enabled = 1;
      setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&enabled), sizeof(enabled));
    });
    server_.new_task_queue = [] { return new httplib::ThreadPool(8, 8, 16); };
    server_.set_payload_max_length(512 * 1024);
    server_.set_read_timeout(3, 0);
    server_.set_write_timeout(2, 0);
    server_.Get("/", [](const auto &, auto &response) {
      std::ifstream file(ament_index_cpp::get_package_share_directory("desktop_server_pkg") + "/web/index.html");
      response.set_content(std::string(std::istreambuf_iterator<char>(file), {}), "text/html; charset=utf-8");
    });
    // callback：客户端轮询 /status，读取当前任务阶段、视频是否可用、以及可调的设置。
    server_.Get("/status", [this, source, mode](const auto &, auto &response) {
      const auto confidence = read_confidence();
      std::lock_guard lock(mutex_);
      auto json = "{\"camera_source\":\"" + source + "\",\"jpeg_quality\":" +
        std::to_string(quality_.load()) + ",\"task\":{\"stage\":\"" + (stage_.empty() ? "preview" : stage_) +
        "\",\"frames\":" + std::to_string(frames_) + "}";
      json += video_available() ? ",\"video_available\":true" : ",\"video_available\":false";
      const char *test = mode == "approach" ? "approach_test_node" : mode == "camera_test" ? "camera_test_node" : "";
      json += ",\"test_node\":\"" + std::string(test) + "\",\"serial\":{\"open\":" +
        (serial_state_.open ? "true" : "false");
      if (serial_state_.team == "red" || serial_state_.team == "blue") {
        json += ",\"team\":\"" + serial_state_.team + "\"";
      }
      if (serial_state_.angle_known) json += ",\"angle_deg\":" + std::to_string(serial_state_.angle_deg);
      if (serial_state_.open && serial_feedback_) {
        json += ",\"command\":" + std::to_string(serial_feedback_->command) +
          ",\"value\":" + std::to_string(serial_feedback_->value);
      }
      json += ",\"seq\":" + std::to_string(serial_seq_);
      json += ",\"frames\":[";
      for (const auto &line : serial_lines_) {
        if (json.back() != '[') json += ',';
        json += '"' + line + '"';
      }
      json += "]}";
      if (confidence) json += ",\"confidence\":" + std::to_string(*confidence);
      response.set_content(json + "}", "application/json");
    });
    // callback：JPEG 质量本节点就能改。置信度要等推理节点确认，之后才回复成功。
    server_.Post("/settings", [this](const auto &request, auto &response) {
      if (!request.has_header("X-Gongchuang-Client") ||
          (!request.has_param("jpeg_quality") && !request.has_param("confidence"))) {
        response.status = 400;
        return;
      }
      try {
        int quality = quality_;
        if (request.has_param("jpeg_quality")) {
          const auto value = request.get_param_value("jpeg_quality");
          size_t end = 0;
          quality = std::stoi(value, &end);
          if (end != value.size() || quality < 25 || quality > 90) throw std::invalid_argument("quality");
        }
        std::optional<double> confidence;
        if (request.has_param("confidence")) {
          const auto value = request.get_param_value("confidence");
          size_t end = 0;
          confidence = std::stod(value, &end);
          if (end != value.size() || !std::isfinite(*confidence) || *confidence < 0 || *confidence > 1)
            throw std::invalid_argument("confidence");
          if (!write_confidence(*confidence)) {
            response.status = 503;
            response.set_content("{\"error\":\"YOLO 未确认设置，刷新状态后重试\"}", "application/json");
            return;
          }
        }
        if (request.has_param("jpeg_quality")) quality_ = quality;
        auto json = "{\"jpeg_quality\":" + std::to_string(quality_.load());
        if (confidence) json += ",\"confidence\":" + std::to_string(*confidence);
        response.set_content(json + "}", "application/json");
      } catch (const std::exception &) {
        response.status = 400;
        response.set_content("{\"error\":\"JPEG 质量须为 25～90，置信度须为 0～1\"}", "application/json");
      }
    });
    // callback：一个 HTTP 请求保持打开，每一段发送一张完整 JPEG。
    server_.Get("/video", [this](const auto &, auto &response) {
      if (viewers_.fetch_add(1) >= 3) {
        --viewers_;
        response.status = 503;
        return;
      }
      response.set_header("Cache-Control", "no-store");
      response.set_chunked_content_provider("multipart/x-mixed-replace; boundary=frame",
        // 每个客户端各自记住上次发过的 JPEG 编号 seen。拿到 JPEG 后先解锁再发网络数据。
        [this, seen = uint64_t{0}](size_t, httplib::DataSink &sink) mutable {
          std::shared_ptr<const std::vector<uchar>> jpeg;
          {
            std::unique_lock lock(mutex_);
            ready_.wait_for(lock, 1s, [&] { return stop_ || (serial_ != seen && video_available()); });
            if (stop_ || !sink.is_writable()) return false;
            if (serial_ == seen || !video_available()) return true;
            seen = serial_;
            jpeg = jpeg_;
          }
          const auto header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
            std::to_string(jpeg->size()) + "\r\n\r\n";
          return sink.write(header.data(), header.size()) &&
            sink.write(reinterpret_cast<const char *>(jpeg->data()), jpeg->size()) && sink.write("\r\n", 2);
        }, [this](bool) { --viewers_; });
    });
    safety_http_ = std::make_unique<SafetyHttp>(*this, server_,
      declare_parameter("yolo_enabled", false), point_x, point_y, [this] {
        std::lock_guard lock(mutex_);
        return stage_ == "track" ? selected_ : Objects::ConstSharedPtr{};
      });
    if (!server_.bind_to_port(host, port)) throw std::runtime_error("无法监听 HTTP 端口");
    // HTTP 接请求和图像压缩各跑各的，ROS 的 spin 在 main() 里继续接话题和参数服务回复。
    http_ = std::thread([this] { server_.listen_after_bind(); });
    encode_ = std::thread([=] { encode_loop(width, height, point_x, point_y); });
    RCLCPP_INFO(get_logger(), "[DesktopServer] http://%s:%d (%s)", host.c_str(), port, source.c_str());
  }

  ~DesktopServer() override {
    stop_ = true;
    ready_.notify_all();
    server_.stop();
    http_.join();
    encode_.join();
  }

private:
  // 只开预览时没有 YOLO 节点。这时用 nullopt 表示拿不到置信度，客户端会禁用对应的设置。
  std::optional<double> read_confidence() {
    if (!read_settings_->service_is_ready()) return std::nullopt;
    auto request = std::make_shared<ReadSettings::Request>();
    request->names = {"yolo.conf_threshold"};
    auto result = read_settings_->async_send_request(request);
    if (result.wait_for(400ms) != std::future_status::ready) {
      read_settings_->remove_pending_request(result);
      return std::nullopt;
    }
    const auto reply = result.get();
    if (reply->values.size() != 1 || reply->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
      return std::nullopt;
    return reply->values[0].double_value;
  }

  bool write_confidence(double confidence) {
    if (!write_settings_->service_is_ready()) return false;
    auto request = std::make_shared<WriteSettings::Request>();
    request->parameters.push_back(rclcpp::Parameter("yolo.conf_threshold", confidence).to_parameter_msg());
    auto result = write_settings_->async_send_request(request);
    if (result.wait_for(1500ms) != std::future_status::ready) {
      write_settings_->remove_pending_request(result);
      return false;
    }
    return result.get()->result.successful;
  }

  // newest_ 还可能留着旧图。两秒没收到新画面，就向客户端报告无视频。
  bool video_available() const {
    return newest_ && std::chrono::steady_clock::now() - frame_received_ < 2s;
  }

  // 每 100 ms 检查一次。先在锁内取出最新画面和状态，缩放、画框和压缩都放在锁外。
  void encode_loop(int width, int height, double point_x, double point_y) {
    Image::ConstSharedPtr last;
    int last_quality = 0;
    bool last_target = false;
    std::string last_stage;
    std::optional<int> last_angle;
    while (!stop_) {
      const auto next = std::chrono::steady_clock::now() + 100ms;
      if (!viewers_.load()) {
        last.reset();
        std::this_thread::sleep_until(next);
        continue;
      }
      Image::ConstSharedPtr image;
      Objects::ConstSharedPtr selected;
      std::string stage;
      std::optional<int> angle;
      {
        std::lock_guard lock(mutex_);
        if (video_available()) image = newest_;
        selected = selected_;
        stage = stage_;
        angle = camera_angle_;
      }
      // 目标框和画面必须是同一帧，避免把前一帧的框画到新画面上。
      const int quality = quality_;
      const bool has_target = image && (stage == "track" || stage == "grab") && selected &&
        selected->header.stamp == image->header.stamp && !selected->items.empty();
      if (image && (image != last || quality != last_quality || has_target != last_target ||
          stage != last_stage || angle != last_angle) && image->encoding == "bgr8" &&
          image->width && image->height && image->step >= image->width * 3 &&
          image->data.size() >= static_cast<size_t>(image->step) * image->height) {
        try {
          cv::Mat frame(image->height, image->width, CV_8UC3,
            const_cast<unsigned char *>(image->data.data()), image->step);
          cv::Mat small;
          const double scale = std::min({1.0, double(width) / frame.cols, double(height) / frame.rows});
          cv::resize(frame, small, {}, scale, scale, cv::INTER_AREA);
          // 参考点按缩略后的比例画，靠近测试时用它对齐物块底边。
          const cv::Point reference(cvRound(point_x * scale), cvRound(point_y * scale));
          if (reference.x >= 0 && reference.y >= 0 && reference.x < small.cols && reference.y < small.rows) {
            cv::drawMarker(small, reference, {80, 255, 80}, cv::MARKER_CROSS, 14, 2);
            cv::circle(small, reference, 4, {80, 255, 80}, 1, cv::LINE_AA);
          }
          std::string display_stage = stage == "search" ? "goto / S1 search" : stage;
          if (stage == "track") display_stage = "goto / S2 track";
          if (stage == "grab_closed") display_stage = "grab / claw closed";
          if (stage == "track" || stage == "grab") {
            if (has_target) {
              const auto &item = selected->items.front();
              const cv::Point upper(cvRound(item.x1*scale), cvRound(item.y1*scale));
              const cv::Point lower(cvRound(item.x2*scale), cvRound(item.y2*scale));
              cv::rectangle(small, upper, lower, {0,255,255}, 2);
              cv::putText(small, "target", {upper.x, std::max(15, upper.y-3)}, cv::FONT_HERSHEY_SIMPLEX, .48, {0,255,255}, 1);
            }
          }
          if (!display_stage.empty()) {
            const cv::Point origin(10, 31);
            cv::putText(small, display_stage, origin, cv::FONT_HERSHEY_SIMPLEX, .95, {0,0,0}, 4, cv::LINE_AA);
            cv::putText(small, display_stage, origin, cv::FONT_HERSHEY_SIMPLEX, .95, {30,210,250}, 2, cv::LINE_AA);
          }
          if (angle) cv::putText(small, "cam_angle: " + std::to_string(*angle), {10,52},
            cv::FONT_HERSHEY_SIMPLEX, .5, {30,210,250}, 1, cv::LINE_AA);
          // 新 JPEG 编好后再换掉缓存。正在发送上一张的客户端仍持有它自己的 shared_ptr。
          auto jpeg = std::make_shared<std::vector<uchar>>();
          if (cv::imencode(".jpg", small, *jpeg, {cv::IMWRITE_JPEG_QUALITY, quality})) {
            {
              std::lock_guard lock(mutex_);
              jpeg_ = jpeg;
              ++serial_;
            }
            ready_.notify_all();
          }
        } catch (const cv::Exception &error) {
          RCLCPP_WARN(get_logger(), "[DesktopServer] %s", error.what());
        }
        last = image;
        last_quality = quality;
        last_target = has_target;
        last_stage = stage;
        last_angle = angle;
      }
      std::this_thread::sleep_until(next);
    }
  }

  httplib::Server server_;
  std::unique_ptr<SafetyHttp> safety_http_;
  // ROS callback、压缩线程和 HTTP 请求共享画面和状态，用同一把锁取值或更新。
  std::mutex mutex_;
  std::condition_variable ready_;
  std::atomic<bool> stop_{false};
  std::atomic<int> quality_{65}, viewers_{0};
  std::chrono::steady_clock::time_point frame_received_{};
  std::thread http_, encode_;
  Image::ConstSharedPtr newest_;
  std::shared_ptr<const std::vector<uchar>> jpeg_;
  uint64_t serial_{0};
  std::string stage_;
  uint64_t frames_{0};
  std::optional<int> camera_angle_;
  Objects::ConstSharedPtr selected_;
  rclcpp::Client<ReadSettings>::SharedPtr read_settings_;
  rclcpp::Client<WriteSettings>::SharedPtr write_settings_;
  RobotState serial_state_;
  uint64_t serial_seq_{0};
  std::deque<std::string> serial_lines_;
  rclcpp::Subscription<SerialFrame>::SharedPtr serial_frames_;
  std::optional<Feedback> serial_feedback_;
  rclcpp::Subscription<Feedback>::SharedPtr feedback_;
  rclcpp::Subscription<RobotState>::SharedPtr robot_;
  rclcpp::Subscription<Objects>::SharedPtr target_;
  rclcpp::Subscription<Image>::SharedPtr input_;
  rclcpp::Subscription<MatchState>::SharedPtr status_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DesktopServer>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("DesktopServer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
