#pragma once

#include "domain/geometry/pose3.h"

namespace lightning::domain::result {

enum class LocalizationStatus {
    kIdle,
    kInitializing,
    kGood,
    kFollowingMotion,
    kFail,
};

struct LocalizationResult {
    double timestamp_s = 0.0;
    geometry::Pose3 pose = geometry::Pose3::Identity();
    bool valid = false;
    LocalizationStatus status = LocalizationStatus::kIdle;

    bool localizer_valid = false;
    bool inlier = false;
    bool motion_consistent = true;
    bool smooth = false;
    double confidence = 0.0;
    double motion_delta_m = 0.0;
};

}  // namespace lightning::domain::result
