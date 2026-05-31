#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "domain/geometry/pose3.h"
#include "domain/sensor/cloud_data.h"

namespace lightning::domain::sensor {

struct ScanSnapshot {
    std::uint64_t stamp_ns = 0;
    std::string frame_id;
    std::string source_id;
    CloudData registered_scan;
    std::vector<CloudData> accumulated_scans;
    geometry::Pose3 odom_pose_hint = geometry::Pose3::Identity();
    bool has_odom_pose_hint = false;

    std::size_t PointCount() const {
        std::size_t count = registered_scan.points.size();
        for (const auto& scan : accumulated_scans) {
            count += scan.points.size();
        }
        return count;
    }

    bool HasMinimumPoints(std::size_t min_points) const { return PointCount() >= min_points; }
};

}  // namespace lightning::domain::sensor
