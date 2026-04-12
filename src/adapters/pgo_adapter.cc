#include "adapters/pgo_adapter.h"

namespace lightning::loc {

namespace {

SE3 ToSE3(const domain::geometry::Pose3& pose) { return SE3(pose.rotation, pose.translation); }

domain::geometry::Pose3 ToPose3(const SE3& pose) {
    domain::geometry::Pose3 result;
    result.translation = pose.translation();
    result.rotation = pose.unit_quaternion();
    return result;
}

domain::result::LocalizationStatus ToDomainStatus(LocalizationStatus status) {
    switch (status) {
        case LocalizationStatus::IDLE:
            return domain::result::LocalizationStatus::kIdle;
        case LocalizationStatus::INITIALIZING:
            return domain::result::LocalizationStatus::kInitializing;
        case LocalizationStatus::GOOD:
            return domain::result::LocalizationStatus::kGood;
        case LocalizationStatus::FOLLOWING_DR:
            return domain::result::LocalizationStatus::kFollowingMotion;
        case LocalizationStatus::FAIL:
            return domain::result::LocalizationStatus::kFail;
    }
    return domain::result::LocalizationStatus::kFail;
}

domain::result::LocalizationResult ToDomainResult(const LocalizationResult& result) {
    domain::result::LocalizationResult converted;
    converted.timestamp_s = result.timestamp_;
    converted.pose = ToPose3(result.pose_);
    converted.valid = result.valid_;
    converted.status = ToDomainStatus(result.status_);
    converted.localizer_valid = result.lidar_loc_valid_;
    converted.inlier = result.lidar_loc_inlier_;
    converted.motion_consistent = result.lidar_loc_odom_error_normal_;
    converted.smooth = result.lidar_loc_smooth_flag_;
    converted.confidence = result.confidence_;
    converted.motion_delta_m = result.lidar_loc_odom_delta_;
    return converted;
}

LocalizationResult ToLegacyResult(const domain::result::LocalizationResult& result) {
    LocalizationResult converted;
    converted.timestamp_ = result.timestamp_s;
    converted.pose_ = ToSE3(result.pose);
    converted.valid_ = result.valid;
    converted.lidar_loc_valid_ = result.localizer_valid;
    converted.lidar_loc_inlier_ = result.inlier;
    converted.lidar_loc_odom_error_normal_ = result.motion_consistent;
    converted.lidar_loc_smooth_flag_ = result.smooth;
    converted.confidence_ = result.confidence;
    converted.lidar_loc_odom_delta_ = result.motion_delta_m;

    switch (result.status) {
        case domain::result::LocalizationStatus::kIdle:
            converted.status_ = LocalizationStatus::IDLE;
            break;
        case domain::result::LocalizationStatus::kInitializing:
            converted.status_ = LocalizationStatus::INITIALIZING;
            break;
        case domain::result::LocalizationStatus::kGood:
            converted.status_ = LocalizationStatus::GOOD;
            break;
        case domain::result::LocalizationStatus::kFollowingMotion:
            converted.status_ = LocalizationStatus::FOLLOWING_DR;
            break;
        case domain::result::LocalizationStatus::kFail:
            converted.status_ = LocalizationStatus::FAIL;
            break;
    }

    return converted;
}

NavState ToLegacyMotionState(const domain::result::MotionEstimate& motion) {
    NavState state;
    state.timestamp_ = motion.timestamp_s;
    state.SetPose(ToSE3(motion.pose));
    state.SetVel(motion.linear_velocity);
    state.confidence_ = motion.confidence;
    state.pose_is_ok_ = motion.valid;
    state.is_parking_ = motion.stationary;
    return state;
}

}  // namespace

PGOAdapter::PGOAdapter() : impl_(std::make_shared<PGO>()) {}

PGOAdapter::PGOAdapter(std::shared_ptr<PGO> impl) : impl_(std::move(impl)) {}

void PGOAdapter::FeedDeadReckoning(const NavState& state) { impl_->ProcessDR(state); }

void PGOAdapter::FeedLidarOdom(const NavState& state) { impl_->ProcessLidarOdom(state); }

void PGOAdapter::FeedLocalization(const LocalizationResult& result) { impl_->ProcessLidarLoc(result); }

void PGOAdapter::SetHighFrequencyOutputCallback(IFusionEngine::OutputCallback cb) {
    impl_->SetHighFrequencyGlobalOutputHandleFunction(std::move(cb));
}

void PGOAdapter::FeedMotionEstimate(const domain::result::MotionEstimate& motion) { FeedLidarOdom(ToLegacyMotionState(motion)); }

void PGOAdapter::FeedLocalizationResult(const domain::result::LocalizationResult& localization) {
    FeedLocalization(ToLegacyResult(localization));
}

void PGOAdapter::SetOutputCallback(domain::contracts::IPoseGraphBackend::OutputCallback callback) {
    backend_output_callback_ = std::move(callback);
    impl_->SetHighFrequencyGlobalOutputHandleFunction([this](const LocalizationResult& result) {
        latest_result_ = ToDomainResult(result);
        if (backend_output_callback_) {
            backend_output_callback_(latest_result_);
        }
    });
}

domain::result::LocalizationResult PGOAdapter::GetLatestResult() const { return latest_result_; }

void PGOAdapter::Reset() { latest_result_ = domain::result::LocalizationResult(); }

void PGOAdapter::SetDebug(bool debug) { impl_->SetDebug(debug); }

}  // namespace lightning::loc
