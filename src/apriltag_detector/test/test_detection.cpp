#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "apriltag_detector/apriltag_detector.hpp"

using apriltag_detector::ApriltagDetector;
using apriltag_detector::DetectorConfig;

TEST(ApriltagDetector, ConstructsForKnownFamily)
{
  DetectorConfig cfg;
  cfg.family = "tag36h11";
  EXPECT_NO_THROW({ ApriltagDetector det(cfg); });
}

TEST(ApriltagDetector, ThrowsForUnknownFamily)
{
  DetectorConfig cfg;
  cfg.family = "not_a_family";
  EXPECT_THROW({ ApriltagDetector det(cfg); }, std::invalid_argument);
}

TEST(ApriltagDetector, BlankImageYieldsNoDetections)
{
  DetectorConfig cfg;
  ApriltagDetector det(cfg);
  cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC1);
  EXPECT_TRUE(det.detect(blank).empty());
}

TEST(ApriltagDetector, EmptyImageIsHandled)
{
  DetectorConfig cfg;
  ApriltagDetector det(cfg);
  cv::Mat empty;
  EXPECT_TRUE(det.detect(empty).empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
