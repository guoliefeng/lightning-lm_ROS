#include "adapters/map_odom_authority_passthrough.h"

namespace lightning::adapters {

namespace {

domain::geometry::Pose3 Inverse(const domain::geometry::Pose3& pose) {
    domain::geometry::Pose3 inverse;
    inverse.rotation = pose.rotation.inverse();
    inverse.translation = inverse.rotation * (-pose.translation);
    return inverse;
}

domain::geometry::Pose3 Compose(const domain::geometry::Pose3& lhs, const domain::geometry::Pose3& rhs) {
    domain::geometry::Pose3 composed;
    composed.rotation = lhs.rotation * rhs.rotation;
    composed.translation = lhs.translation + lhs.rotation * rhs.translation;
    return composed;
}

}  // namespace

bool PassthroughMapOdomAuthority::UpdateFromLocalization(
    const domain::result::LocalizationResult& localization,
    const domain::geometry::Pose3& odom_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frozen_ || !localization.valid) {
        return false;
    }

    map_to_odom_ = Compose(localization.pose, Inverse(odom_pose));
    return true;
}

domain::geometry::Pose3 PassthroughMapOdomAuthority::GetMapToOdom() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_to_odom_;
}

void PassthroughMapOdomAuthority::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    map_to_odom_ = domain::geometry::Pose3::Identity();
    frozen_ = false;
}

void PassthroughMapOdomAuthority::Freeze() {
    std::lock_guard<std::mutex> lock(mutex_);
    frozen_ = true;
}

void PassthroughMapOdomAuthority::Unfreeze() {
    std::lock_guard<std::mutex> lock(mutex_);
    frozen_ = false;
}

bool PassthroughMapOdomAuthority::IsFrozen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frozen_;
}

}  // namespace lightning::adapters
