#pragma once

#include <string>

#include "domain/geometry/pose3.h"

namespace lightning::domain::result {

enum class AlignmentStatus {
    kNotReady,
    kConverged,
    kRejected,
    kLost,
    kFailed,
};

struct AlignmentResult {
    geometry::Pose3 pose = geometry::Pose3::Identity();
    double confidence = 0.0;
    bool success = false;
    AlignmentStatus status = AlignmentStatus::kNotReady;
    std::string message;
};

}  // namespace lightning::domain::result
