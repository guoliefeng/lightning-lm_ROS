#include "adapters/legacy_localizer_relocalization_adapter.h"

#include <utility>

#include "interfaces/localizer.h"

namespace lightning::adapters {

namespace {

domain::geometry::Pose3 ToPose3(const SE3& pose) {
    domain::geometry::Pose3 converted;
    converted.translation = pose.translation();
    converted.rotation = pose.unit_quaternion();
    return converted;
}

const domain::sensor::CloudData& SelectRegisteredScan(const domain::sensor::ScanSnapshot& snapshot) {
    return snapshot.registered_scan;
}

}  // namespace

LegacyLocalizerRelocalizationAdapter::LegacyLocalizerRelocalizationAdapter(
    std::shared_ptr<domain::contracts::ILocalizer> localizer)
    : localizer_(std::move(localizer)) {}

bool LegacyLocalizerRelocalizationAdapter::IsReady(const domain::sensor::ScanSnapshot& snapshot) const {
    return !SelectRegisteredScan(snapshot).points.empty();
}

domain::result::AlignmentResult LegacyLocalizerRelocalizationAdapter::Initialize(
    const domain::sensor::ScanSnapshot& snapshot,
    const domain::result::MapState*) {
    return ProcessWithLocalizer(snapshot);
}

domain::result::AlignmentResult LegacyLocalizerRelocalizationAdapter::Track(
    const domain::sensor::ScanSnapshot& snapshot,
    const domain::geometry::Pose3&) {
    return ProcessWithLocalizer(snapshot);
}

void LegacyLocalizerRelocalizationAdapter::Reset() {}

CloudPtr LegacyLocalizerRelocalizationAdapter::BuildLegacyCloud(
    const domain::sensor::ScanSnapshot& snapshot) const {
    const auto& scan = SelectRegisteredScan(snapshot);
    if (scan.points.empty()) {
        return nullptr;
    }

    CloudPtr cloud(new PointCloudType());
    cloud->header.stamp = snapshot.stamp_ns;
    cloud->is_dense = scan.is_dense;
    cloud->reserve(scan.points.size());
    for (const auto& point : scan.points) {
        PointType converted;
        converted.x = point.x;
        converted.y = point.y;
        converted.z = point.z;
        converted.intensity = point.intensity;
        converted.time = point.relative_time_s;
        cloud->push_back(converted);
    }
    return cloud;
}

domain::result::AlignmentResult LegacyLocalizerRelocalizationAdapter::ProcessWithLocalizer(
    const domain::sensor::ScanSnapshot& snapshot) {
    domain::result::AlignmentResult alignment;
    if (!localizer_) {
        alignment.status = domain::result::AlignmentStatus::kFailed;
        alignment.message = "legacy localizer adapter has no localizer";
        return alignment;
    }

    if (!IsReady(snapshot)) {
        alignment.status = domain::result::AlignmentStatus::kNotReady;
        alignment.message = "registered scan is empty";
        return alignment;
    }

    auto cloud = BuildLegacyCloud(snapshot);
    if (!cloud) {
        alignment.status = domain::result::AlignmentStatus::kNotReady;
        alignment.message = "failed to build legacy cloud";
        return alignment;
    }

    const bool processed = localizer_->ProcessKeyframeScan(cloud);
    const auto result = localizer_->GetLocalizationResult();

    alignment.pose = ToPose3(result.pose_);
    alignment.confidence = result.confidence_;
    alignment.success = processed && result.valid_ && result.lidar_loc_valid_;
    alignment.status =
        alignment.success ? domain::result::AlignmentStatus::kConverged : domain::result::AlignmentStatus::kRejected;
    if (!processed) {
        alignment.status = domain::result::AlignmentStatus::kFailed;
        alignment.message = "legacy localizer rejected keyframe scan";
    } else if (!alignment.success) {
        alignment.message = "legacy localizer did not produce a valid localization result";
    }
    return alignment;
}

}  // namespace lightning::adapters
