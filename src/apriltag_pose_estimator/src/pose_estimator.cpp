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

double quad_area(const std::array<cv::Point2d, 4> & c)
{
  double a = 0.0;
  for (int i = 0; i < 4; ++i) {
    const auto & p = c[i];
    const auto & q = c[(i + 1) % 4];
    a += p.x * q.y - q.x * p.y;
  }
  return std::abs(a) * 0.5;
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

std::vector<SingleTagPose> PoseEstimator::estimate_single_tag_solutions(
  const std::array<cv::Point2d, 4> & corners, double size,
  const CameraIntrinsics & intr) const
{
  const double h = size / 2.0;
  const std::vector<cv::Point3d> object_points = {
    cv::Point3d(-h, h, 0.0),
    cv::Point3d(h, h, 0.0),
    cv::Point3d(h, -h, 0.0),
    cv::Point3d(-h, -h, 0.0)};
  const std::vector<cv::Point2d> image_points = {
    corners[3], corners[2], corners[1], corners[0]};

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  std::vector<SingleTagPose> out;
  try {
    cv::solvePnPGeneric(
      object_points, image_points, intr.K, intr.dist, rvecs, tvecs, false,
      cv::SOLVEPNP_IPPE_SQUARE);
  } catch (const cv::Exception &) {
    return out;
  }

  for (size_t i = 0; i < rvecs.size(); ++i) {
    const cv::Vec3d rv(
      rvecs[i].at<double>(0), rvecs[i].at<double>(1), rvecs[i].at<double>(2));
    const cv::Vec3d tv(
      tvecs[i].at<double>(0), tvecs[i].at<double>(1), tvecs[i].at<double>(2));
    SingleTagPose s;
    s.T_cam_tag = rtvec_to_isometry(rv, tv);
    s.reproj_rms = reprojection_rms(object_points, image_points, intr, rv, tv);
    s.valid = true;
    out.push_back(s);
  }
  return out;
}

bool PoseEstimator::within_bounds(const Eigen::Vector3d & p) const
{
  return std::abs(p.x()) <= bounds_.x_limit &&
         std::abs(p.y()) <= bounds_.y_limit &&
         p.z() >= bounds_.z_min && p.z() <= bounds_.z_max;
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

  // Combined solve over all tags, accepted if it stays within the map bounds.
  if (result.num_tags >= 2) {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    if (cv::solvePnP(
        object_points, image_points, intr.K, intr.dist, rvec, tvec, false,
        cv::SOLVEPNP_SQPNP))
    {
      const Eigen::Isometry3d T_map_camera = rtvec_to_isometry(rvec, tvec).inverse();
      const double reproj =
        reprojection_rms(object_points, image_points, intr, rvec, tvec);
      if (reproj <= cov_.max_reproj_rms && within_bounds(T_map_camera.translation())) {
        double mean_range = 0.0;
        for (const auto * det : used) {
          mean_range +=
            (T_map_camera.translation() - map.get(det->id)->pose_map.translation()).norm();
        }
        mean_range /= static_cast<double>(used.size());
        result.T_map_camera = T_map_camera;
        result.reproj_rms = reproj;
        result.covariance = build_covariance(reproj, result.num_tags, mean_range);
        result.valid = true;
        return result;
      }
    }
  }

  // Otherwise use the largest tag, keeping the IPPE solution inside the bounds.
  const DetectionInput * best = nullptr;
  double best_area = -1.0;
  for (const auto * det : used) {
    const double a = quad_area(det->corners);
    if (a > best_area) {
      best_area = a;
      best = det;
    }
  }
  const auto info = map.get(best->id);
  const auto solutions = estimate_single_tag_solutions(best->corners, info->size, intr);

  const SingleTagPose * chosen = nullptr;
  for (const auto & s : solutions) {
    const Eigen::Isometry3d T_map_camera = info->pose_map * s.T_cam_tag.inverse();
    if (!within_bounds(T_map_camera.translation())) {
      continue;
    }
    if (chosen == nullptr || s.reproj_rms < chosen->reproj_rms) {
      chosen = &s;
    }
  }
  if (chosen == nullptr || chosen->reproj_rms > cov_.max_reproj_rms) {
    result.valid = false;
    return result;
  }

  result.T_map_camera = info->pose_map * chosen->T_cam_tag.inverse();
  result.reproj_rms = chosen->reproj_rms;
  result.num_tags = 1;
  const double range = chosen->T_cam_tag.translation().norm();
  result.covariance = build_covariance(chosen->reproj_rms, 1, range);
  result.valid = true;
  return result;
}

Eigen::Matrix<double, 6, 6> PoseEstimator::build_covariance(
  double reproj_rms, int num_tags, double mean_range) const
{
  const double n = std::max(1, num_tags);
  const double pos_std =
    (cov_.base_pos_std + cov_.pos_px_scale * reproj_rms +
     cov_.pos_dist_scale * mean_range) / std::sqrt(n);
  double rot_std =
    (cov_.base_rot_std + cov_.rot_px_scale * reproj_rms +
     cov_.rot_dist_scale * mean_range) / std::sqrt(n);
  if (num_tags <= 1) {
    rot_std *= cov_.single_tag_rot_factor;
  }

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
