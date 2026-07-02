#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "domain/geometry/pose3.h"

namespace lightning::domain::sensor {

struct OdometryData {
    std::uint64_t stamp_ns = 0;
    std::string sensor_id;
    std::string frame_id;
    std::string child_frame_id;

    geometry::Pose3 pose = geometry::Pose3::Identity();
    bool pose_valid = false;

    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    bool twist_valid = false;
};

}  // namespace lightning::domain::sensor
