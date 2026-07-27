#include "map_runtime/map_frame_anchor.h"

#include <glog/logging.h>

#include <cmath>

#include "io/yaml_io.h"

namespace lightning {

namespace {

bool TryGetBool(const YAML_IO& yaml, const std::string& node, const std::string& key, bool default_value) {
    try {
        return yaml.GetValue<bool>(node, key);
    } catch (...) {
        return default_value;
    }
}

std::string TryGetString(const YAML_IO& yaml, const std::string& node, const std::string& key,
                         const std::string& default_value) {
    try {
        return yaml.GetValue<std::string>(node, key);
    } catch (...) {
        return default_value;
    }
}

Vec3d TryGetVec3(const YAML_IO& yaml, const std::string& node, const std::string& key, const Vec3d& default_value) {
    try {
        const auto values = yaml.GetValue<std::vector<double>>(node, key);
        if (values.size() < 3) {
            return default_value;
        }
        return Vec3d(values[0], values[1], values[2]);
    } catch (...) {
        return default_value;
    }
}

}  // namespace

SE3 MapFrameAnchor::PoseFromRpyTranslation(const Vec3d& translation, const Vec3d& rpy_deg) {
    const double roll = rpy_deg.x() * M_PI / 180.0;
    const double pitch = rpy_deg.y() * M_PI / 180.0;
    const double yaw = rpy_deg.z() * M_PI / 180.0;
    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    const Quatd q = yaw_angle * pitch_angle * roll_angle;
    return SE3(q, translation);
}

SE3 MapFrameAnchor::PoseFromYamlNode(const YAML_IO& yaml, const std::string& node, const std::string& subnode) {
    try {
        const double x = yaml.GetValue<double>(node, subnode, "x");
        const double y = yaml.GetValue<double>(node, subnode, "y");
        const double z = yaml.GetValue<double>(node, subnode, "z");
        const double qx = yaml.GetValue<double>(node, subnode, "qx");
        const double qy = yaml.GetValue<double>(node, subnode, "qy");
        const double qz = yaml.GetValue<double>(node, subnode, "qz");
        const double qw = yaml.GetValue<double>(node, subnode, "qw");
        Quatd q(qw, qx, qy, qz);
        q.normalize();
        return SE3(q, Vec3d(x, y, z));
    } catch (...) {
        const Vec3d translation = TryGetVec3(yaml, node, subnode + "_translation", Vec3d::Zero());
        const Vec3d rpy_deg = TryGetVec3(yaml, node, subnode + "_rpy_deg", Vec3d::Zero());
        return PoseFromRpyTranslation(translation, rpy_deg);
    }
}

bool MapFrameAnchor::LoadFromYaml(const std::string& yaml_path) {
    YAML_IO yaml(yaml_path);
    if (!yaml.IsOpened()) {
        LOG(WARNING) << "map_frame anchor: failed to open yaml " << yaml_path;
        return false;
    }
    return LoadFromYaml(yaml);
}

bool MapFrameAnchor::LoadFromYaml(const YAML_IO& yaml) {
    enabled_ = false;
    T_ins_to_map_ = SE3();

    enabled_ = TryGetBool(yaml, "map_frame", "enabled", false);
    if (!enabled_) {
        LOG(INFO) << "map_frame anchor disabled";
        return true;
    }

    const std::string mode = TryGetString(yaml, "map_frame", "mode", "fixed");
    if (mode == "anchor_pair") {
        const SE3 T_ins_anchor = PoseFromYamlNode(yaml, "map_frame", "ins_anchor");
        const SE3 T_map_anchor = PoseFromYamlNode(yaml, "map_frame", "map_anchor");
        T_ins_to_map_ = T_map_anchor * T_ins_anchor.inverse();
        LOG(INFO) << "map_frame anchor_pair: ins_anchor t=" << T_ins_anchor.translation().transpose()
                  << ", map_anchor t=" << T_map_anchor.translation().transpose();
    } else {
        const Vec3d translation = TryGetVec3(yaml, "map_frame", "ins_to_map_translation", Vec3d::Zero());
        const Vec3d rpy_deg = TryGetVec3(yaml, "map_frame", "ins_to_map_rotation_rpy_deg", Vec3d::Zero());
        T_ins_to_map_ = PoseFromRpyTranslation(translation, rpy_deg);
        LOG(INFO) << "map_frame fixed: trans=" << translation.transpose() << ", rpy_deg=" << rpy_deg.transpose();
    }

    LOG(INFO) << "map_frame anchor enabled, T_ins_to_map t=" << T_ins_to_map_.translation().transpose()
              << ", yaw_deg=" << std::atan2(T_ins_to_map_.rotationMatrix()(1, 0), T_ins_to_map_.rotationMatrix()(0, 0)) *
                     180.0 / M_PI;
    return true;
}

SE3 MapFrameAnchor::TransformInsToMap(const SE3& pose_ins) const {
    if (!enabled_) {
        return pose_ins;
    }
    return T_ins_to_map_ * pose_ins;
}

domain::geometry::Pose3 MapFrameAnchor::TransformPoseInsToMap(const domain::geometry::Pose3& pose_ins) const {
    const SE3 transformed = TransformInsToMap(SE3(pose_ins.rotation, pose_ins.translation));
    domain::geometry::Pose3 out;
    out.translation = transformed.translation();
    out.rotation = transformed.unit_quaternion();
    return out;
}

Vec3d MapFrameAnchor::TransformPointInsToMap(const Vec3d& point_ins) const {
    if (!enabled_) {
        return point_ins;
    }
    return T_ins_to_map_ * point_ins;
}

}  // namespace lightning
