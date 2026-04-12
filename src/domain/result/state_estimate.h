#pragma once

#include <Eigen/Core>

#include "domain/geometry/pose3.h"

namespace lightning::domain::result {

struct StateEstimate {
    double timestamp_s = 0.0;
    geometry::Pose3 pose = geometry::Pose3::Identity();
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    double confidence = 0.0;
    bool valid = false;
};

}  // namespace lightning::domain::result
