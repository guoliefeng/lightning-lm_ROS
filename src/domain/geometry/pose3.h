#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lightning::domain::geometry {

struct Pose3 {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

    static Pose3 Identity() { return Pose3(); }
};

}  // namespace lightning::domain::geometry
