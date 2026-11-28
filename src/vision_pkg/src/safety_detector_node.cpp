#include "vision_pkg/safety_detector.hpp"

#include <limits>
#include <memory>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/safety_detection.hpp>
#include <robot_interfaces/msg/safety_regions.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

class SafetyDetectorNode : public rclcpp::Node {
public:
  SafetyDetectorNode() : Node("safety_detector_node") {
    for (std::size_t i = 0; i < vision_pkg::hsv_names.size(); ++i) {
      const auto &range = vision_pkg::default_hsv[i];
      const auto values = declare_parameter("hsv." + std::string(vision_pkg::hsv_names[i]),
        std::vector<int64_t>(range.begin(), range.end()));
      vision_pkg::HsvRange parsed{};
      if (!read_range(values, parsed)) throw std::invalid_argument("Invalid HSV bounds");
    }
    hsv_callback_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto &parameter : parameters) {
        if (parameter.get_name().rfind("hsv.", 0) != 0) continue;
        vision_pkg::HsvRange parsed{};
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY ||
            !read_range(parameter.as_integer_array(), parsed)) {
          result.successful = false;
          result.reason = "HSV: [Hmin,Smin,Vmin,Hmax,Smax,Vmax], H 0-179, S/V 0-255";
          break;
        }
      }
      return result;
    });
    publisher_ = create_publisher<Detection>("vision/safety_detection", rclcpp::SensorDataQoS().keep_last(1));
    regions_ = create_publisher<robot_interfaces::msg::SafetyRegions>("vision/safety_regions_raw", rclcpp::SensorDataQoS());
    paired_ = declare_parameter("paired", false);
    set_enabled(paired_);
    service_ = create_service<SetBool>("vision/safety_detector/enable",
      [this](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        if (paired_ && !request->data) {
          response->success = false;
          response->message = "perception requires safety regions";
          return;
        }
        set_enabled(request->data);
        response->success = true;
        response->message = request->data ? "enabled" : "disabled";
      });
  }

private:
  using Image = sensor_msgs::msg::Image;
  using Detection = robot_interfaces::msg::SafetyDetection;
  using SetBool = std_srvs::srv::SetBool;

  static bool read_range(const std::vector<int64_t> &values, vision_pkg::HsvRange &range) {
    if (values.size() != range.size()) return false;
    for (std::size_t i = 0; i < range.size(); ++i) {
      if (values[i] < 0 || values[i] > 255) return false;
      range[i] = static_cast<int>(values[i]);
    }
    return vision_pkg::valid_hsv(range);
  }

  void set_enabled(bool enabled) {
    if (!enabled) {
      camera_.reset();
    } else if (!camera_) {
      camera_ = create_subscription<Image>(paired_ ? "vision/inference_image" : "camera/image", rclcpp::SensorDataQoS(),
        [this](Image::UniquePtr image) { detect(*image); });
    }
  }

  void detect(Image &image) {
    constexpr auto max_dimension = std::numeric_limits<int>::max();
    if (image.encoding != sensor_msgs::image_encodings::BGR8 || !image.width || !image.height ||
        image.width > max_dimension || image.height > max_dimension ||
        image.step < 3ULL * image.width || image.data.size() != 1ULL * image.step * image.height) return;

    try {
      const cv::Mat frame(static_cast<int>(image.height), static_cast<int>(image.width),
                          CV_8UC3, image.data.data(), image.step);
      auto ranges = vision_pkg::default_hsv;
      for (std::size_t i = 0; i < ranges.size(); ++i)
        read_range(get_parameter("hsv." + std::string(vision_pkg::hsv_names[i])).as_integer_array(), ranges[i]);
      const auto safety = detector_.detect(frame, ranges);
      Detection detection;
      detection.header = image.header;
      detection.flag = safety.flag;
      detection.detected = safety.status.detected;
      detection.full = safety.status.full;
      detection.ok = safety.status.ok;
      for (const auto &point : safety.polygon) {
        auto &pixel = detection.polygon.emplace_back();
        pixel.x = static_cast<float>(point.x);
        pixel.y = static_cast<float>(point.y);
      }
      publisher_->publish(detection);

      // 先保留已有的主轮廓供投放使用，再屏蔽它找另一方的安全区。
      robot_interfaces::msg::SafetyRegions regions;
      regions.header = image.header;
      auto add = [&regions](const vision_pkg::Safety &zone) {
        if (zone.flag != "r_safety" && zone.flag != "b_safety") return;
        auto &region = regions.regions.emplace_back();
        region.name = zone.flag;
        for (const auto &point : zone.polygon) {
          auto &pixel = region.polygon.emplace_back();
          pixel.x = static_cast<float>(point.x);
          pixel.y = static_cast<float>(point.y);
        }
      };
      add(safety);
      if (paired_ && safety.polygon.size() >= 3) {
        auto other_frame = frame.clone();
        cv::fillConvexPoly(other_frame, safety.polygon, cv::Scalar::all(0));
        const auto other = detector_.detect(other_frame, ranges);
        if (other.flag != safety.flag) add(other);
      }
      regions_->publish(regions);
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "[SafetyDetector] %s", error.what());
    }
  }

  vision_pkg::SafetyDetector detector_;
  OnSetParametersCallbackHandle::SharedPtr hsv_callback_;
  rclcpp::Publisher<Detection>::SharedPtr publisher_;
  rclcpp::Publisher<robot_interfaces::msg::SafetyRegions>::SharedPtr regions_;
  bool paired_ = false;
  rclcpp::Subscription<Image>::SharedPtr camera_;
  rclcpp::Service<SetBool>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyDetectorNode>());
  rclcpp::shutdown();
}
