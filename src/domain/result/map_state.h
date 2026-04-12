#pragma once

#include <cstddef>
#include <string>

#include "domain/geometry/pose3.h"

namespace lightning::domain::result {

struct MapState {
    std::string map_id;
    std::string frame_id;
    geometry::Pose3 anchor_pose = geometry::Pose3::Identity();
    bool loaded = false;
    bool target_ready = false;
    bool dirty = false;
    std::size_t static_point_count = 0;
    std::size_t dynamic_point_count = 0;
};

}  // namespace lightning::domain::result
