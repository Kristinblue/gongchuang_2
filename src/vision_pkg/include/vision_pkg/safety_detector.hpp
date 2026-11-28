#pragma once

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace vision_pkg {

using HsvRange = std::array<int, 6>;
using SafetyHsv = std::array<HsvRange, 3>;
enum HsvColor { Purple, Red, Blue };
inline constexpr std::array<const char *, 3> hsv_names{"purple", "red", "blue"};
inline constexpr SafetyHsv default_hsv{{{125, 100, 25, 155, 255, 255},
                                        {155, 70, 35, 2, 255, 255},
                                        {98, 22, 35, 145, 150, 255}}};

bool valid_hsv(const HsvRange &range);

struct Safety {
  std::string flag{"safety"};
  struct Status {
    bool detected{false};
    bool full{false};
    bool ok{false};
  } status;
  std::vector<cv::Point> polygon;
  std::vector<cv::Point> object_polygon;
};

struct SafetyDebug {
  cv::Rect roi;
  cv::Mat purple, red, blue;
};

class SafetyDetector {
public:
  Safety detect(const cv::Mat &image, const SafetyHsv &ranges = default_hsv, SafetyDebug *debug = nullptr,
                bool extend_for_objects = false) const;
};

}  // namespace vision_pkg
