#pragma once

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <iomanip>
#include <mutex>
#include <sstream>

#include <httplib.h>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <robot_interfaces/msg/objects.hpp>
#include <sensor_msgs/msg/image.hpp>

// 主界面接收原图及同帧检测坐标，阈值以 ROS 安全区节点为准。
class SafetyHttp {
  using Image = sensor_msgs::msg::Image;
  using Objects = robot_interfaces::msg::Objects;

public:
  SafetyHttp(rclcpp::Node &node, httplib::Server &server, bool yolo_enabled, double point_x, double point_y,
             std::function<Objects::ConstSharedPtr()> target, std::function<std::string()> stage)
    : node_(node), yolo_enabled_(yolo_enabled), point_x_(point_x), point_y_(point_y),
      target_(std::move(target)), stage_(std::move(stage)) {
    read_ = node_.create_client<Read>("safety_detector_node/get_parameters");
    write_ = node_.create_client<Write>("safety_detector_node/set_parameters_atomically");
    server.Get("/safety/hsv", [this](const auto &, auto &response) { read_hsv(response); });
    server.Post("/safety/hsv", [this](const auto &request, auto &response) { write_hsv(request, response); });
    server.Get("/safety/frame", [this](const auto &request, auto &response) { frame(request, response); });
    idle_ = node_.create_wall_timer(std::chrono::seconds(1), [this] {
      std::lock_guard lock(mutex_);
      if (Clock::now() - requested_ > std::chrono::seconds(2)) {
        camera_.reset();
        detector_.reset();
        pending_image_.reset();
        pending_boxes_.reset();
        image_.reset();
        boxes_.reset();
      }
    });
  }

private:
  using Read = rcl_interfaces::srv::GetParameters;
  using Write = rcl_interfaces::srv::SetParametersAtomically;
  using Clock = std::chrono::steady_clock;
  using Ranges = std::array<std::vector<int64_t>, 3>;
  static constexpr std::array<const char *, 3> names{"purple", "red", "blue"};

  static void reply_hsv(const Ranges &ranges, httplib::Response &response) {
    std::string json = "{";
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      if (i) json += ',';
      json += "\"" + std::string(names[i]) + "\":[";
      for (std::size_t j = 0; j < ranges[i].size(); ++j) {
        if (j) json += ',';
        json += std::to_string(ranges[i][j]);
      }
      json += ']';
    }
    response.set_header("Cache-Control", "no-store");
    response.set_content(json + "}", "application/json");
  }

  template<class Service>
  static typename Service::Response::SharedPtr call(
      const std::shared_ptr<rclcpp::Client<Service>> &client,
      const typename Service::Request::SharedPtr &request) {
    if (!client->service_is_ready()) return {};
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(1500)) == std::future_status::ready) return future.get();
    client->remove_pending_request(future);
    return {};
  }

  void read_hsv(httplib::Response &response) {
    auto request = std::make_shared<Read::Request>();
    for (auto name : names) request->names.push_back("hsv." + std::string(name));
    auto result = call<Read>(read_, request);
    Ranges ranges;
    if (!result || result->values.size() != ranges.size()) {
      response.status = 503;
      return;
    }
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      ranges[i] = result->values[i].integer_array_value;
      if (ranges[i].size() != 6) { response.status = 503; return; }
    }
    reply_hsv(ranges, response);
  }

  void write_hsv(const httplib::Request &http, httplib::Response &response) {
    if (!http.has_header("X-Gongchuang-Client")) { response.status = 400; return; }
    auto request = std::make_shared<Write::Request>();
    Ranges ranges;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      std::istringstream input(http.get_param_value(names[i]));
      std::string token;
      while (std::getline(input, token, ',')) {
        int64_t number{};
        const auto end = token.data() + token.size();
        const auto parsed = std::from_chars(token.data(), end, number);
        if (parsed.ec != std::errc{} || parsed.ptr != end || number < 0 || number > 255) {
          response.status = 400;
          return;
        }
        ranges[i].push_back(number);
      }
      if (ranges[i].size() != 6) { response.status = 400; return; }
      request->parameters.push_back(rclcpp::Parameter("hsv." + std::string(names[i]), ranges[i]).to_parameter_msg());
    }
    auto result = call<Write>(write_, request);
    if (!result) { response.status = 503; return; }
    if (!result->result.successful) { response.status = 400; return; }
    reply_hsv(ranges, response);
  }

  void pair_frame() {
    if (!pending_image_ || !pending_boxes_ || pending_image_->header != pending_boxes_->header) return;
    image_ = std::move(pending_image_);
    boxes_ = std::move(pending_boxes_);
    received_ = Clock::now();
    ++serial_;
  }

  void subscribe_camera() {
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    camera_ = node_.create_subscription<Image>(yolo_enabled_ ? "vision/inference_image" : "camera/image", qos,
      [this](Image::ConstSharedPtr frame) {
        std::lock_guard guard(mutex_);
        if (yolo_enabled_) {
          pending_image_ = std::move(frame);
          pair_frame();
        } else {
          image_ = std::move(frame);
          received_ = Clock::now();
          ++serial_;
        }
      });
    if (yolo_enabled_) {
      detector_ = node_.create_subscription<Objects>("vision/detections", qos, [this](Objects::ConstSharedPtr detections) {
        std::lock_guard guard(mutex_);
        pending_boxes_ = std::move(detections);
        pair_frame();
      });
    }
  }

  static void write_box(std::ostream &json, const robot_interfaces::msg::Object &box) {
    auto name = box.name.substr(0, 32);
    for (auto &ch : name)
      if (static_cast<unsigned char>(ch) < 32 || static_cast<unsigned char>(ch) > 126) ch = '_';
    json << '[' << std::quoted(name) << ',' << box.x1 << ',' << box.y1 << ',' << box.x2 << ',' << box.y2 << ']';
  }

  std::string frame_overlays(const Image &image, const Objects::ConstSharedPtr &boxes) const {
    std::ostringstream json;
    json << "{\"boxes\":[";
    int count = 0;
    if (boxes) {
      for (const auto &box : boxes->items) {
        if (!std::isfinite(box.x1) || !std::isfinite(box.y1) || !std::isfinite(box.x2) || !std::isfinite(box.y2)) continue;
        if (count == 64) break;
        if (count++) json << ',';
        write_box(json, box);
      }
    }
    json << "],\"point\":[" << point_x_ << ',' << point_y_ << ']';
    const auto selected = target_();
    if (selected && selected->header == image.header && !selected->items.empty()) {
      const auto &box = selected->items.front();
      if (std::isfinite(box.x1) && std::isfinite(box.y1) && std::isfinite(box.x2) && std::isfinite(box.y2)) {
        json << ",\"target\":";
        write_box(json, box);
      }
    }
    json << ",\"stage\":" << std::quoted(stage_()) << '}';
    return json.str();
  }

  void frame(const httplib::Request &request, httplib::Response &response) {
    Image::ConstSharedPtr image;
    Objects::ConstSharedPtr boxes;
    uint64_t serial = 0;
    {
      std::lock_guard lock(mutex_);
      requested_ = Clock::now();
      if (!camera_) subscribe_camera();
      if (requested_ - received_ < std::chrono::seconds(2)) {
        image = image_;
        boxes = boxes_;
        serial = serial_;
      }
    }
    if (!image || image->encoding != "bgr8" || !image->width || !image->height ||
        image->step < 3ULL * image->width || image->data.size() != 1ULL * image->step * image->height) {
      response.status = 503;
      return;
    }
    const bool jpeg = request.get_param_value("format") == "jpeg";
    const auto etag = "\"" + std::to_string(image->header.stamp.sec) + '-' +
      std::to_string(image->header.stamp.nanosec) + '-' + std::to_string(serial) + (jpeg ? "-jpeg\"" : "-png\"");
    response.set_header("Cache-Control", "no-store");
    response.set_header("ETag", etag);
    response.set_header("X-Gongchuang-Vision", frame_overlays(*image, boxes));
    if (request.get_header_value("If-None-Match") == etag) {
      response.status = 304;
      return;
    }
    const cv::Mat pixels(image->height, image->width, CV_8UC3,
                         const_cast<unsigned char *>(image->data.data()), image->step);
    std::vector<unsigned char> encoded;
    const std::vector<int> params = jpeg ? std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 65}
                                         : std::vector<int>{cv::IMWRITE_PNG_COMPRESSION, 1};
    if (!cv::imencode(jpeg ? ".jpg" : ".png", pixels, encoded, params)) {
      response.status = 500;
      return;
    }
    response.set_content(reinterpret_cast<const char *>(encoded.data()), encoded.size(), jpeg ? "image/jpeg" : "image/png");
  }

  rclcpp::Node &node_;
  bool yolo_enabled_;
  double point_x_, point_y_;
  std::function<Objects::ConstSharedPtr()> target_;
  std::function<std::string()> stage_;
  rclcpp::Client<Read>::SharedPtr read_;
  rclcpp::Client<Write>::SharedPtr write_;
  rclcpp::Subscription<Image>::SharedPtr camera_;
  rclcpp::Subscription<Objects>::SharedPtr detector_;
  rclcpp::TimerBase::SharedPtr idle_;
  std::mutex mutex_;
  Clock::time_point requested_{}, received_{};
  Image::ConstSharedPtr image_, pending_image_;
  Objects::ConstSharedPtr boxes_, pending_boxes_;
  uint64_t serial_{0};
};
