#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace lightning::domain::sensor {

struct ImuData {
    std::uint64_t stamp_ns = 0;
    std::string sensor_id;
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};

}  // namespace lightning::domain::sensor
