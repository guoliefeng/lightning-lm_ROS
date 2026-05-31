#include "adapters/global_initializer_passthrough.h"

#include <algorithm>

namespace lightning::adapters {

PassthroughGlobalInitializer::PassthroughGlobalInitializer() = default;

PassthroughGlobalInitializer::PassthroughGlobalInitializer(Options options) : options_(options) {}

bool PassthroughGlobalInitializer::IsReady(const domain::sensor::ScanSnapshot& snapshot) const {
    return snapshot.HasMinimumPoints(options_.min_points);
}

domain::result::AlignmentResult PassthroughGlobalInitializer::Initialize(
    const domain::sensor::ScanSnapshot& snapshot,
    const domain::result::MapState*) {
    domain::result::AlignmentResult result;
    if (!IsReady(snapshot)) {
        result.status = domain::result::AlignmentStatus::kNotReady;
        result.message = "not enough accumulated points for global initialization";
        return result;
    }

    result.pose = snapshot.has_odom_pose_hint ? snapshot.odom_pose_hint : domain::geometry::Pose3::Identity();
    result.confidence = std::min(1.0, static_cast<double>(snapshot.PointCount()) / static_cast<double>(options_.min_points));
    result.success = true;
    result.status = domain::result::AlignmentStatus::kConverged;
    return result;
}

void PassthroughGlobalInitializer::Reset() {}

}  // namespace lightning::adapters
