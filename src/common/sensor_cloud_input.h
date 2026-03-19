#pragma once

#include <cstdint>
#include <functional>

#include "common/point_def.h"

namespace lightning {

class PointCloudPreprocess;

struct SensorCloudInput {
    using Converter = std::function<void(PointCloudPreprocess&, CloudPtr&)>;

    std::uint64_t stamp_ns = 0;
    CloudPtr cloud = nullptr;
    Converter converter;

    bool Empty() const { return cloud == nullptr && !converter; }
};

}  // namespace lightning
