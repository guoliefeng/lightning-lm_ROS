#include "application/system/relocalization_coordinator.h"

#include <utility>

namespace lightning::application::system {

namespace {

domain::result::AlignmentResult MakeFailure(domain::result::AlignmentStatus status, const std::string& message) {
    domain::result::AlignmentResult result;
    result.success = false;
    result.status = status;
    result.message = message;
    return result;
}

domain::result::LocalizationStatus ToLocalizationStatus(domain::result::AlignmentStatus status) {
    switch (status) {
        case domain::result::AlignmentStatus::kConverged:
            return domain::result::LocalizationStatus::kGood;
        case domain::result::AlignmentStatus::kNotReady:
            return domain::result::LocalizationStatus::kInitializing;
        case domain::result::AlignmentStatus::kLost:
            return domain::result::LocalizationStatus::kFollowingMotion;
        case domain::result::AlignmentStatus::kRejected:
        case domain::result::AlignmentStatus::kFailed:
            return domain::result::LocalizationStatus::kFail;
    }
    return domain::result::LocalizationStatus::kFail;
}

}  // namespace

RelocalizationCoordinator::RelocalizationCoordinator(
    Options options,
    std::shared_ptr<domain::contracts::IGlobalInitializer> global_initializer,
    std::shared_ptr<domain::contracts::ILocalTracker> local_tracker,
    std::shared_ptr<domain::contracts::IMapOdomAuthority> map_odom_authority)
    : options_(std::move(options)),
      global_initializer_(std::move(global_initializer)),
      local_tracker_(std::move(local_tracker)),
      map_odom_authority_(std::move(map_odom_authority)) {}

void RelocalizationCoordinator::SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) {
    std::lock_guard<std::mutex> lock(mutex_);
    event_sink_ = std::move(sink);
}

domain::result::RelocalizationState RelocalizationCoordinator::GetState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

domain::result::LocalizationMode RelocalizationCoordinator::GetMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    switch (state_) {
        case domain::result::RelocalizationState::kGlobalInitializing:
        case domain::result::RelocalizationState::kAccumulating:
            return domain::result::LocalizationMode::kGlobalInitialization;
        case domain::result::RelocalizationState::kTracking:
            return domain::result::LocalizationMode::kLocalTracking;
        case domain::result::RelocalizationState::kIdle:
        case domain::result::RelocalizationState::kLost:
        case domain::result::RelocalizationState::kFailed:
        case domain::result::RelocalizationState::kRecovering:
            return domain::result::LocalizationMode::kUninitialized;
    }
    return domain::result::LocalizationMode::kUninitialized;
}

domain::geometry::Pose3 RelocalizationCoordinator::GetMapToOdom() const {
    return map_odom_authority_ ? map_odom_authority_->GetMapToOdom() : domain::geometry::Pose3::Identity();
}

domain::result::AlignmentResult RelocalizationCoordinator::ProcessScan(
    const domain::sensor::ScanSnapshot& snapshot) {
    domain::sensor::ScanSnapshot working_snapshot;
    domain::result::RelocalizationState state;
    domain::geometry::Pose3 initial_pose;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!global_initializer_ || !local_tracker_ || !map_odom_authority_) {
            TransitionToLocked(domain::result::RelocalizationState::kFailed);
            return MakeFailure(domain::result::AlignmentStatus::kFailed,
                               "relocalization coordinator dependencies are incomplete");
        }

        state = state_;
        if (state == domain::result::RelocalizationState::kIdle ||
            state == domain::result::RelocalizationState::kAccumulating ||
            state == domain::result::RelocalizationState::kLost ||
            state == domain::result::RelocalizationState::kRecovering) {
            if (!snapshot.registered_scan.points.empty()) {
                accumulated_scans_.push_back(snapshot.registered_scan);
            }
            if (!HasEnoughAccumulationLocked()) {
                TransitionToLocked(domain::result::RelocalizationState::kAccumulating);
                return MakeFailure(domain::result::AlignmentStatus::kNotReady,
                                   "waiting for enough accumulated scan structure");
            }
            TransitionToLocked(domain::result::RelocalizationState::kGlobalInitializing);
            working_snapshot = BuildAccumulatedSnapshotLocked(snapshot);
        } else {
            working_snapshot = snapshot;
            initial_pose = last_tracking_pose_;
        }
    }

    domain::result::AlignmentResult alignment;
    if (state == domain::result::RelocalizationState::kTracking) {
        alignment = local_tracker_->Track(working_snapshot, initial_pose);
    } else {
        alignment = global_initializer_->Initialize(working_snapshot, nullptr);
    }

    const auto localization = ToLocalizationResult(alignment, working_snapshot);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (alignment.success) {
            last_tracking_pose_ = alignment.pose;
            map_odom_authority_->UpdateFromLocalization(localization, ResolveOdomPoseHint(working_snapshot));
            accumulated_scans_.clear();
            TransitionToLocked(domain::result::RelocalizationState::kTracking);
        } else {
            TransitionToLocked(domain::result::RelocalizationState::kLost);
            if (options_.freeze_map_to_odom_when_lost) {
                map_odom_authority_->Freeze();
            }
        }
    }

    if (alignment.success) {
        PublishLocalizationResult(localization);
    }
    return alignment;
}

void RelocalizationCoordinator::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    accumulated_scans_.clear();
    last_tracking_pose_ = domain::geometry::Pose3::Identity();
    if (global_initializer_) {
        global_initializer_->Reset();
    }
    if (local_tracker_) {
        local_tracker_->Reset();
    }
    if (map_odom_authority_) {
        map_odom_authority_->Reset();
    }
    TransitionToLocked(domain::result::RelocalizationState::kIdle);
}

void RelocalizationCoordinator::FreezeMapToOdom() {
    if (map_odom_authority_) {
        map_odom_authority_->Freeze();
    }
}

void RelocalizationCoordinator::UnfreezeMapToOdom() {
    if (map_odom_authority_) {
        map_odom_authority_->Unfreeze();
    }
}

bool RelocalizationCoordinator::HasEnoughAccumulationLocked() const {
    if (accumulated_scans_.size() < options_.min_accumulated_scans) {
        return false;
    }
    std::size_t point_count = 0;
    for (const auto& scan : accumulated_scans_) {
        point_count += scan.points.size();
    }
    return point_count >= options_.min_accumulated_points;
}

domain::sensor::ScanSnapshot RelocalizationCoordinator::BuildAccumulatedSnapshotLocked(
    const domain::sensor::ScanSnapshot& latest) const {
    auto snapshot = latest;
    snapshot.accumulated_scans = accumulated_scans_;
    return snapshot;
}

domain::result::LocalizationResult RelocalizationCoordinator::ToLocalizationResult(
    const domain::result::AlignmentResult& alignment,
    const domain::sensor::ScanSnapshot& snapshot) const {
    domain::result::LocalizationResult result;
    result.timestamp_s = static_cast<double>(snapshot.stamp_ns) * 1e-9;
    result.pose = alignment.pose;
    result.valid = alignment.success;
    result.status = ToLocalizationStatus(alignment.status);
    result.localizer_valid = alignment.success;
    result.confidence = alignment.confidence;
    return result;
}

domain::geometry::Pose3 RelocalizationCoordinator::ResolveOdomPoseHint(
    const domain::sensor::ScanSnapshot& snapshot) const {
    return snapshot.has_odom_pose_hint ? snapshot.odom_pose_hint : domain::geometry::Pose3::Identity();
}

void RelocalizationCoordinator::TransitionToLocked(domain::result::RelocalizationState state) {
    if (state_ == state) {
        return;
    }
    state_ = state;
    if (event_sink_) {
        event_sink_->OnRelocalizationState(state_);
    }
}

void RelocalizationCoordinator::PublishLocalizationResult(
    const domain::result::LocalizationResult& result) const {
    std::shared_ptr<domain::contracts::IEventSink> sink;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sink = event_sink_;
    }
    if (sink) {
        sink->OnLocalizationResult(result);
    }
}

}  // namespace lightning::application::system
