#include "adapters/local_tracker_passthrough.h"

namespace lightning::adapters {

domain::result::AlignmentResult PassthroughLocalTracker::Track(const domain::sensor::ScanSnapshot& snapshot,
                                                               const domain::geometry::Pose3& initial_pose) {
    domain::result::AlignmentResult result;
    if (snapshot.registered_scan.points.empty()) {
        result.status = domain::result::AlignmentStatus::kLost;
        result.message = "registered scan is empty";
        return result;
    }

    result.pose = snapshot.has_odom_pose_hint ? snapshot.odom_pose_hint : initial_pose;
    result.confidence = 1.0;
    result.success = true;
    result.status = domain::result::AlignmentStatus::kConverged;
    return result;
}

void PassthroughLocalTracker::Reset() {}

}  // namespace lightning::adapters
