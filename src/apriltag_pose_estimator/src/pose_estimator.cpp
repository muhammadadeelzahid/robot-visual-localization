#include "apriltag_pose_estimator/pose_estimator.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>

namespace apriltag_pose_estimator
{

namespace
{

Eigen::Isometry3d rtvec_to_isometry(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T.linear()(r, c) = R(r, c);
    }
    T.translation()(r) = tvec(r);
  }
  return T;
}

double reprojection_rms(
  const std::vector<cv::Point3d> & object_points,
  const std::vector<cv::Point2d> & image_points,
  const CameraIntrinsics & intr, const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  std::vector<cv::Point2d> projected;
  cv::projectPoints(object_points, rvec, tvec, intr.K, intr.dist, projected);

  double sum_sq = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    const double dx = projected[i].x - image_points[i].x;
    const double dy = projected[i].y - image_points[i].y;
    sum_sq += dx * dx + dy * dy;
  }
  return std::sqrt(sum_sq / static_cast<double>(projected.size()));
}

}  // namespace

PoseEstimator::PoseEstimator(const CovarianceModel & cov)
: cov_(cov)
{
}

std::array<cv::Point3d, 4> PoseEstimator::tag_object_points(double size)
{
  const double h = size / 2.0;
  // AprilTag corner order: bottom-left, bottom-right, top-right, top-left,
  // in the tag frame (+x right, +y up, +z out of the face).
  return {
    cv::Point3d(-h, -h, 0.0),
    cv::Point3d(h, -h, 0.0),
    cv::Point3d(h, h, 0.0),
    cv::Point3d(-h, h, 0.0)};
}

bool PoseEstimator::estimate_single_tag(
  const std::array<cv::Point2d, 4> & corners, double size,
  const CameraIntrinsics & intr, SingleTagPose & out) const
{
  const double h = size / 2.0;

  // SOLVEPNP_IPPE_SQUARE expects corners in the order
  // top-left, top-right, bottom-right, bottom-left with +y up.
  const std::vector<cv::Point3d> object_points = {
    cv::Point3d(-h, h, 0.0),
    cv::Point3d(h, h, 0.0),
    cv::Point3d(h, -h, 0.0),
    cv::Point3d(-h, -h, 0.0)};

  // Reorder AprilTag corners (BL, BR, TR, TL) into IPPE order.
  const std::vector<cv::Point2d> image_points = {
    corners[3], corners[2], corners[1], corners[0]};

  cv::Vec3d rvec;
  cv::Vec3d tvec;
  const bool ok = cv::solvePnP(
    object_points, image_points, intr.K, intr.dist, rvec, tvec, false,
    cv::SOLVEPNP_IPPE_SQUARE);
  if (!ok) {
    out.valid = false;
    return false;
  }

  out.id = -1;
  out.T_cam_tag = rtvec_to_isometry(rvec, tvec);
  out.reproj_rms = reprojection_rms(object_points, image_points, intr, rvec, tvec);
  out.valid = true;
  return true;
}

CameraPoseEstimate PoseEstimator::estimate_camera_pose(
  const std::vector<DetectionInput> & detections, const TagMap & map,
  const CameraIntrinsics & intr) const
{
  CameraPoseEstimate result;

  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;
  std::vector<const DetectionInput *> used;

  for (const auto & det : detections) {
    const auto info = map.get(det.id);
    if (!info) {
      continue;
    }
    const auto tag_pts = tag_object_points(info->size);
    for (int k = 0; k < 4; ++k) {
      const Eigen::Vector3d p_tag(tag_pts[k].x, tag_pts[k].y, tag_pts[k].z);
      const Eigen::Vector3d p_map = info->pose_map * p_tag;
      object_points.emplace_back(p_map.x(), p_map.y(), p_map.z());
      image_points.push_back(det.corners[k]);
    }
    used.push_back(&det);
  }

  result.num_tags = static_cast<int>(used.size());
  if (result.num_tags == 0) {
    result.valid = false;
    return result;
  }

  if (result.num_tags == 1) {
    // Single tag: solve in the tag frame, then compose with the tag's map pose.
    const auto info = map.get(used.front()->id);
    SingleTagPose single;
    if (!estimate_single_tag(used.front()->corners, info->size, intr, single)) {
      result.valid = false;
      return result;
    }
    result.T_map_camera = info->pose_map * single.T_cam_tag.inverse();
    result.reproj_rms = single.reproj_rms;
    result.covariance = build_covariance(single.reproj_rms, 1);
    result.valid = true;
    return result;
  }

  // Multiple tags: one combined solve over all corners expressed in the map frame.
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  const bool ok = cv::solvePnP(
    object_points, image_points, intr.K, intr.dist, rvec, tvec, false,
    cv::SOLVEPNP_SQPNP);
  if (!ok) {
    result.valid = false;
    return result;
  }

  // solvePnP gives T_camera_map; invert for the camera pose in the map frame.
  const Eigen::Isometry3d T_camera_map = rtvec_to_isometry(rvec, tvec);
  result.T_map_camera = T_camera_map.inverse();
  result.reproj_rms = reprojection_rms(object_points, image_points, intr, rvec, tvec);
  result.covariance = build_covariance(result.reproj_rms, result.num_tags);
  result.valid = true;
  return result;
}

Eigen::Matrix<double, 6, 6> PoseEstimator::build_covariance(
  double reproj_rms, int num_tags) const
{
  const double n = std::max(1, num_tags);
  const double pos_std =
    (cov_.base_pos_std + cov_.pos_px_scale * reproj_rms) / std::sqrt(n);
  const double rot_std =
    (cov_.base_rot_std + cov_.rot_px_scale * reproj_rms) / std::sqrt(n);

  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  const double pos_var = pos_std * pos_std;
  const double rot_var = rot_std * rot_std;
  cov(0, 0) = pos_var;
  cov(1, 1) = pos_var;
  cov(2, 2) = pos_var;
  cov(3, 3) = rot_var;
  cov(4, 4) = rot_var;
  cov(5, 5) = rot_var;
  return cov;
}

}  // namespace apriltag_pose_estimator
