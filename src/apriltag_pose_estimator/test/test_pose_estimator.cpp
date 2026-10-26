#include <gtest/gtest.h>

#include <array>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>

#include "apriltag_pose_estimator/pose_estimator.hpp"
#include "apriltag_pose_estimator/tag_map.hpp"

using namespace apriltag_pose_estimator;

namespace
{

CameraIntrinsics make_intrinsics()
{
  CameraIntrinsics intr;
  intr.K = cv::Matx33d(600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0);
  return intr;
}

cv::Vec3d eigen_to_rvec(const Eigen::Isometry3d & T)
{
  cv::Matx33d R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = T.rotation()(r, c);
    }
  }
  cv::Vec3d rvec;
  cv::Rodrigues(R, rvec);
  return rvec;
}

cv::Vec3d eigen_to_tvec(const Eigen::Isometry3d & T)
{
  return cv::Vec3d(T.translation().x(), T.translation().y(), T.translation().z());
}

// Project a tag's corners (in some reference frame) given the transform from that
// frame into the camera optical frame.
std::array<cv::Point2d, 4> project_corners(
  const std::array<cv::Point3d, 4> & object_points,
  const Eigen::Isometry3d & T_cam_ref, const CameraIntrinsics & intr)
{
  std::vector<cv::Point3d> obj(object_points.begin(), object_points.end());
  std::vector<cv::Point2d> img;
  cv::projectPoints(
    obj, eigen_to_rvec(T_cam_ref), eigen_to_tvec(T_cam_ref), intr.K, intr.dist, img);
  std::array<cv::Point2d, 4> out;
  for (int i = 0; i < 4; ++i) {
    out[i] = img[i];
  }
  return out;
}

double translation_error(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return (a.translation() - b.translation()).norm();
}

double rotation_error(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  const Eigen::Matrix3d r = a.rotation().transpose() * b.rotation();
  const double trace = std::min(3.0, std::max(-1.0, r.trace()));
  return std::acos((trace - 1.0) / 2.0);
}

}  // namespace

TEST(PoseEstimator, SingleTagRecoversKnownPose)
{
  const auto intr = make_intrinsics();
  PoseEstimator estimator;
  const double size = 0.16;

  // Tag one metre in front of the camera, slightly rotated.
  Eigen::Isometry3d T_cam_tag = Eigen::Isometry3d::Identity();
  T_cam_tag.translation() = Eigen::Vector3d(0.05, -0.02, 1.0);
  T_cam_tag.linear() =
    Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const auto obj = PoseEstimator::tag_object_points(size);
  const auto corners = project_corners(obj, T_cam_tag, intr);

  SingleTagPose out;
  ASSERT_TRUE(estimator.estimate_single_tag(corners, size, intr, out));
  EXPECT_LT(translation_error(out.T_cam_tag, T_cam_tag), 1e-3);
  EXPECT_LT(rotation_error(out.T_cam_tag, T_cam_tag), 1e-3);
  EXPECT_LT(out.reproj_rms, 1e-3);
}

TEST(PoseEstimator, MultiTagRecoversCameraPoseInMap)
{
  const auto intr = make_intrinsics();
  PoseEstimator estimator;
  const double size = 0.16;

  TagMap map;
  map.set_frame_id("map");

  // Two tags on a wall at x = 2 m, facing back toward the origin (-x normal).
  Eigen::Isometry3d wall = Eigen::Isometry3d::Identity();
  wall.linear() =
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

  TagInfo t0;
  t0.id = 0;
  t0.size = size;
  t0.pose_map = wall;
  t0.pose_map.translation() = Eigen::Vector3d(2.0, -0.5, 0.5);
  map.insert(t0);

  TagInfo t1;
  t1.id = 1;
  t1.size = size;
  t1.pose_map = wall;
  t1.pose_map.translation() = Eigen::Vector3d(2.0, 0.5, 0.5);
  map.insert(t1);

  // Camera at the origin looking down +x. Camera optical +z must point along
  // map +x, so rotate optical frame accordingly.
  Eigen::Isometry3d T_map_cam = Eigen::Isometry3d::Identity();
  T_map_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.5);
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(0, -1, 0);   // optical x -> map -y
  R.col(1) = Eigen::Vector3d(0, 0, -1);   // optical y -> map -z
  R.col(2) = Eigen::Vector3d(1, 0, 0);    // optical z -> map +x
  T_map_cam.linear() = R;
  const Eigen::Isometry3d T_cam_map = T_map_cam.inverse();

  std::vector<DetectionInput> detections;
  for (const auto & [id, info] : map.tags()) {
    std::array<cv::Point3d, 4> obj_map;
    const auto obj_tag = PoseEstimator::tag_object_points(info.size);
    for (int k = 0; k < 4; ++k) {
      const Eigen::Vector3d p =
        info.pose_map * Eigen::Vector3d(obj_tag[k].x, obj_tag[k].y, obj_tag[k].z);
      obj_map[k] = cv::Point3d(p.x(), p.y(), p.z());
    }
    DetectionInput in;
    in.id = id;
    in.corners = project_corners(obj_map, T_cam_map, intr);
    detections.push_back(in);
  }

  const auto estimate = estimator.estimate_camera_pose(detections, map, intr);
  ASSERT_TRUE(estimate.valid);
  EXPECT_EQ(estimate.num_tags, 2);
  EXPECT_LT(translation_error(estimate.T_map_camera, T_map_cam), 1e-2);
  EXPECT_LT(rotation_error(estimate.T_map_camera, T_map_cam), 1e-2);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
