#ifndef APRILTAG_DETECTOR__DETECTION_HPP_
#define APRILTAG_DETECTOR__DETECTION_HPP_

#include <array>
#include <string>

#include <opencv2/core.hpp>

namespace tag_detector
{

/// A single AprilTag detection in image space.
struct Detection
{
  int id{-1};
  std::string family;
  int hamming{0};
  double decision_margin{0.0};

  /// Tag centre in pixels.
  cv::Point2d center;

  /// Tag corners in pixels, ordered counter-clockwise starting from the
  /// bottom-left corner of the tag as defined by the AprilTag library
  /// (corners[0] = (-1,-1), [1] = (1,-1), [2] = (1,1), [3] = (-1,1) in tag frame).
  std::array<cv::Point2d, 4> corners;
};

}  // namespace tag_detector

#endif  // APRILTAG_DETECTOR__DETECTION_HPP_
