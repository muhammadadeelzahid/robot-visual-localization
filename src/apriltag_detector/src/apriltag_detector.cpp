#include "apriltag_detector/apriltag_detector.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag16h5.h>
#include <tagStandard41h12.h>
#include <tagCircle21h7.h>
}

namespace tag_detector
{

namespace
{

apriltag_family_t * create_family(const std::string & name)
{
  if (name == "tag36h11") { return tag36h11_create(); }
  if (name == "tag25h9") { return tag25h9_create(); }
  if (name == "tag16h5") { return tag16h5_create(); }
  if (name == "tagStandard41h12") { return tagStandard41h12_create(); }
  if (name == "tagCircle21h7") { return tagCircle21h7_create(); }
  throw std::invalid_argument("Unsupported AprilTag family: " + name);
}

void destroy_family(const std::string & name, apriltag_family_t * family)
{
  if (family == nullptr) { return; }
  if (name == "tag36h11") { tag36h11_destroy(family); }
  else if (name == "tag25h9") { tag25h9_destroy(family); }
  else if (name == "tag16h5") { tag16h5_destroy(family); }
  else if (name == "tagStandard41h12") { tagStandard41h12_destroy(family); }
  else if (name == "tagCircle21h7") { tagCircle21h7_destroy(family); }
}

}  // namespace

ApriltagDetector::ApriltagDetector(const DetectorConfig & config)
: config_(config)
{
  family_ = create_family(config_.family);

  detector_ = apriltag_detector_create();
  apriltag_detector_add_family(detector_, family_);

  detector_->quad_decimate = static_cast<float>(config_.quad_decimate);
  detector_->quad_sigma = static_cast<float>(config_.quad_sigma);
  detector_->nthreads = config_.nthreads;
  detector_->refine_edges = config_.refine_edges;
  detector_->decode_sharpening = config_.decode_sharpening;
}

ApriltagDetector::~ApriltagDetector()
{
  if (detector_ != nullptr) {
    apriltag_detector_destroy(detector_);
    detector_ = nullptr;
  }
  destroy_family(config_.family, family_);
  family_ = nullptr;
}

std::vector<Detection> ApriltagDetector::detect(const cv::Mat & gray) const
{
  if (gray.empty()) {
    return {};
  }

  cv::Mat mono = gray;
  if (gray.type() != CV_8UC1) {
    cv::cvtColor(gray, mono, cv::COLOR_BGR2GRAY);
  }
  if (!mono.isContinuous()) {
    mono = mono.clone();
  }

  image_u8_t image{
    mono.cols,
    mono.rows,
    static_cast<int>(mono.step),
    mono.data};

  zarray_t * raw = apriltag_detector_detect(detector_, &image);

  std::vector<Detection> detections;
  const int count = zarray_size(raw);
  detections.reserve(count);

  for (int i = 0; i < count; ++i) {
    apriltag_detection_t * det = nullptr;
    zarray_get(raw, i, &det);

    if (det->decision_margin < config_.min_decision_margin) {
      continue;
    }

    Detection d;
    d.id = det->id;
    d.family = config_.family;
    d.hamming = det->hamming;
    d.decision_margin = det->decision_margin;
    d.center = cv::Point2d(det->c[0], det->c[1]);
    for (int k = 0; k < 4; ++k) {
      d.corners[k] = cv::Point2d(det->p[k][0], det->p[k][1]);
    }
    detections.push_back(d);
  }

  apriltag_detections_destroy(raw);
  return detections;
}

}  // namespace tag_detector
