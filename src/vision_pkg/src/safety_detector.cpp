#include "vision_pkg/safety_detector.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace vision_pkg {
namespace {

constexpr double min_color_fraction = .12;
constexpr int frame_edge_margin = 3;

cv::Mat hsv_mask(const cv::Mat &hsv, const HsvRange &range) {
  const auto [h_min, s_min, v_min, h_max, s_max, v_max] = range;
  cv::Mat mask;
  if (h_min <= h_max) {
    cv::inRange(hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
    return mask;
  }
  // 红色跨过 H=0，先筛 S/V，再扣掉两段红色之间的色相。
  cv::Mat hue, middle;
  cv::inRange(hsv, cv::Scalar(0, s_min, v_min), cv::Scalar(179, s_max, v_max), mask);
  cv::extractChannel(hsv, hue, 0);
  cv::inRange(hue, cv::Scalar(h_max + 1), cv::Scalar(h_min - 1), middle);
  cv::subtract(mask, middle, mask);
  return mask;
}

// 用最大紫块定位，只合并附近够大的紫块。
std::vector<cv::Point> find_hull(const cv::Mat &mask) {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) return {};

  std::vector<double> areas;
  for (const auto &contour : contours) areas.push_back(cv::contourArea(contour));
  const auto largest = std::max_element(areas.begin(), areas.end());
  if (*largest < 40) return {};
  const auto anchor = static_cast<std::size_t>(largest - areas.begin());
  const auto box = cv::boundingRect(contours[anchor]);
  const auto gap = .45 * std::max(box.width, box.height);
  const auto min_area = std::max(40., .12 * *largest);
  auto points = contours[anchor];
  for (std::size_t i = 0; i < contours.size(); ++i) {
    if (i == anchor || areas[i] < min_area) continue;
    const auto &contour = contours[i];
    // 始终相对最大紫块判断距离，避免临近碎片把范围一路扩大。
    const auto nearby = std::any_of(contour.begin(), contour.end(), [&](const cv::Point &p) {
      return p.x >= box.x - gap && p.x <= box.br().x + gap &&
             p.y >= box.y - gap && p.y <= box.br().y + gap;
    });
    if (nearby) points.insert(points.end(), contour.begin(), contour.end());
  }
  std::vector<cv::Point> hull;
  cv::convexHull(points, hull);
  return hull;
}

void classify_color(const cv::Mat &image, const cv::Mat &purple, const cv::Rect &roi,
                    const SafetyHsv &ranges, Safety &safety, SafetyDebug *debug) {
  cv::Mat interior = cv::Mat::zeros(roi.size(), CV_8UC1);
  cv::fillPoly(interior, std::vector<std::vector<cv::Point>>{safety.polygon},
               cv::Scalar(255), cv::LINE_8, 0, -roi.tl());
  const int area = cv::countNonZero(interior);
  cv::subtract(interior, purple(roi), interior);
  const int visible = cv::countNonZero(interior);
  safety.status.full = safety.status.full && visible >= .14 * area;
  if (!visible) return;

  cv::Mat hsv;
  cv::cvtColor(image(roi), hsv, cv::COLOR_BGR2HSV);
  auto red = hsv_mask(hsv, ranges[Red]);
  auto blue = hsv_mask(hsv, ranges[Blue]);
  cv::bitwise_and(red, interior, red);
  cv::bitwise_and(blue, interior, blue);
  if (debug) {
    debug->red = red;
    debug->blue = blue;
  }
  int red_count = cv::countNonZero(red), blue_count = cv::countNonZero(blue);
  const double min_pixels = area * min_color_fraction;
  if (red_count < min_pixels) red_count = 0;
  if (blue_count < min_pixels) blue_count = 0;
  if (red_count != blue_count) {
    safety.flag = red_count > blue_count ? "r_safety" : "b_safety";
    safety.status.ok = safety.status.full;
  }
}

// 紫框被画面裁掉时，凸包可能把同一片红蓝地面切在外面。
// 只补与原轮廓有交集、颜色一致的连通地面，补出的范围仅用于过滤物块。
void extend_object_polygon(const cv::Mat &hsv, const SafetyHsv &ranges, Safety &safety) {
  const auto color = safety.flag == "r_safety" ? Red : Blue;
  auto floor = hsv_mask(hsv, ranges[color]);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(floor, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const auto bounds = cv::boundingRect(safety.polygon);
  cv::Mat inside = cv::Mat::zeros(bounds.size(), CV_8UC1);
  cv::fillPoly(inside, std::vector<std::vector<cv::Point>>{safety.polygon},
               cv::Scalar(255), cv::LINE_8, 0, -bounds.tl());
  cv::Mat overlap = cv::Mat::zeros(bounds.size(), CV_8UC1);
  const std::vector<cv::Point> *best = nullptr;
  int best_area = 39;
  for (const auto &contour : contours) {
    if (cv::contourArea(contour) < 40 || (bounds & cv::boundingRect(contour)).empty()) continue;
    overlap.setTo(0);
    cv::fillPoly(overlap, std::vector<std::vector<cv::Point>>{contour},
                 cv::Scalar(255), cv::LINE_8, 0, -bounds.tl());
    cv::bitwise_and(overlap, inside, overlap);
    const int area = cv::countNonZero(overlap);
    if (area > best_area) {
      best = &contour;
      best_area = area;
    }
  }
  if (!best) return;
  auto points = safety.polygon;
  points.insert(points.end(), best->begin(), best->end());
  cv::convexHull(points, safety.object_polygon);
}

}  // namespace

bool valid_hsv(const HsvRange &range) {
  const auto [h_min, s_min, v_min, h_max, s_max, v_max] = range;
  return h_min >= 0 && h_min <= 179 && h_max >= 0 && h_max <= 179 &&
         s_min >= 0 && s_min <= s_max && s_max <= 255 &&
         v_min >= 0 && v_min <= v_max && v_max <= 255;
}

Safety SafetyDetector::detect(const cv::Mat &image, const SafetyHsv &ranges, SafetyDebug *debug,
                              bool extend_for_objects) const {
  Safety safety;
  cv::Mat blurred, hsv, closed;
  cv::GaussianBlur(image, blurred, {3, 3}, 0);
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
  const auto purple = hsv_mask(hsv, ranges[Purple]);
  static const auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
  cv::morphologyEx(purple, closed, cv::MORPH_CLOSE, kernel);
  if (debug) *debug = {};
  const auto hull = find_hull(closed);
  if (hull.empty() || cv::contourArea(hull) < std::max(650., image.total() * .001)) return safety;

  const auto roi = cv::boundingRect(hull);
  if (debug) {
    debug->roi = roi;
    debug->purple = closed(roi);
  }
  cv::approxPolyDP(hull, safety.polygon, .004 * cv::arcLength(hull, true), true);
  const auto margin = std::min({roi.x, roi.y, image.cols - roi.br().x, image.rows - roi.br().y});
  const auto size = cv::minAreaRect(hull).size;
  safety.status.detected = true;
  safety.status.full = margin >= frame_edge_margin &&
                       std::min(size.width, size.height) >= .075 * std::max(size.width, size.height);
  classify_color(image, purple, roi, ranges, safety, debug);
  if (extend_for_objects && margin < frame_edge_margin && safety.flag != "safety")
    extend_object_polygon(hsv, ranges, safety);
  return safety;
}

}  // namespace vision_pkg
