#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace lightning::domain::sensor {

enum class GnssFixStatus {
    kNoFix,
    kFix,
    kSbasFix,
    kGbasFix,
};

struct GnssData {
    std::uint64_t stamp_ns = 0;
    std::string sensor_id;

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;

    GnssFixStatus status = GnssFixStatus::kNoFix;
    Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Zero();
    bool covariance_valid = false;
};

}  // namespace lightning::domain::sensor
