#ifndef APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_
#define APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "apriltag_detector/detection.hpp"

// Forward declarations of the AprilTag C types to keep this header free of the
// C library includes.
struct apriltag_detector;
struct apriltag_family;

namespace tag_detector
{

/// Configuration for the underlying AprilTag detector.
struct DetectorConfig
{
  std::string family{"tag36h11"};
  double quad_decimate{2.0};
  double quad_sigma{0.0};
  int nthreads{1};
  bool refine_edges{true};
  double decode_sharpening{0.25};
  /// Reject detections whose decision margin is below this threshold.
  double min_decision_margin{30.0};
};

/// Thin, RAII wrapper around the AprilTag C library. Not copyable; the
/// underlying detector owns heap resources that must be freed exactly once.
class ApriltagDetector
{
public:
  explicit ApriltagDetector(const DetectorConfig & config);
  ~ApriltagDetector();

  ApriltagDetector(const ApriltagDetector &) = delete;
  ApriltagDetector & operator=(const ApriltagDetector &) = delete;

  /// Detect tags in a grayscale (CV_8UC1) image.
  std::vector<Detection> detect(const cv::Mat & gray) const;

  const DetectorConfig & config() const { return config_; }

private:
  DetectorConfig config_;
  ::apriltag_detector * detector_{nullptr};
  ::apriltag_family * family_{nullptr};
};

}  // namespace tag_detector

#endif  // APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_
