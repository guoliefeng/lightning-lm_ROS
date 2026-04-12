#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lightning::domain::sensor {

struct CloudPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    double relative_time_s = 0.0;
};

struct CloudData {
    std::uint64_t stamp_ns = 0;
    std::string sensor_id;
    std::string frame_id;
    bool is_dense = false;
    std::vector<CloudPoint> points;
};

}  // namespace lightning::domain::sensor
