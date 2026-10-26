#include "apriltag_pose_estimator/tag_map.hpp"

#include <yaml-cpp/yaml.h>

namespace apriltag_pose_estimator
{

bool TagMap::load_from_yaml(const std::string & path, std::string & error)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    error = "failed to open tag map '" + path + "': " + e.what();
    return false;
  }

  if (root["frame_id"]) {
    frame_id_ = root["frame_id"].as<std::string>();
  }
  if (root["default_size"]) {
    default_size_ = root["default_size"].as<double>();
  }

  const YAML::Node tags = root["tags"];
  if (!tags || !tags.IsSequence()) {
    error = "tag map '" + path + "' has no 'tags' sequence";
    return false;
  }

  tags_.clear();
  for (const auto & node : tags) {
    TagInfo info;
    if (!node["id"]) {
      error = "a tag entry is missing 'id'";
      return false;
    }
    info.id = node["id"].as<int>();
    info.size = node["size"] ? node["size"].as<double>() : default_size_;

    Eigen::Vector3d p(0.0, 0.0, 0.0);
    if (node["position"]) {
      const auto pos = node["position"];
      p = Eigen::Vector3d(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>());
    }

    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    if (node["orientation"]) {
      const auto o = node["orientation"];
      // Stored as [x, y, z, w].
      q = Eigen::Quaterniond(
        o[3].as<double>(), o[0].as<double>(), o[1].as<double>(), o[2].as<double>());
      q.normalize();
    }

    info.pose_map = Eigen::Isometry3d::Identity();
    info.pose_map.linear() = q.toRotationMatrix();
    info.pose_map.translation() = p;

    tags_[info.id] = info;
  }

  if (tags_.empty()) {
    error = "tag map '" + path + "' contains no tags";
    return false;
  }

  return true;
}

std::optional<TagInfo> TagMap::get(int id) const
{
  auto it = tags_.find(id);
  if (it == tags_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace apriltag_pose_estimator
