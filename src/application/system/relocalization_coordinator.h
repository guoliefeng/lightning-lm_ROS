#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

#include "domain/contracts/event_sink.h"
#include "domain/contracts/global_initializer.h"
#include "domain/contracts/local_tracker.h"
#include "domain/contracts/map_odom_authority.h"
#include "domain/result/alignment_result.h"
#include "domain/result/localization_result.h"
#include "domain/result/relocalization_state.h"
#include "domain/sensor/cloud_data.h"
#include "domain/sensor/scan_snapshot.h"

namespace lightning::application::system {

class RelocalizationCoordinator {
   public:
    struct Options {
        std::size_t min_accumulated_scans = 1;
        std::size_t min_accumulated_points = 500;
        bool freeze_map_to_odom_when_lost = true;
    };

    RelocalizationCoordinator(Options options,
                              std::shared_ptr<domain::contracts::IGlobalInitializer> global_initializer,
                              std::shared_ptr<domain::contracts::ILocalTracker> local_tracker,
                              std::shared_ptr<domain::contracts::IMapOdomAuthority> map_odom_authority);

    void SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink);
    domain::result::RelocalizationState GetState() const;
    domain::result::LocalizationMode GetMode() const;
    domain::result::LocalizationResult GetLatestLocalizationResult() const;
    domain::geometry::Pose3 GetMapToOdom() const;

    domain::result::AlignmentResult ProcessScan(const domain::sensor::ScanSnapshot& snapshot);
    void Reset();
    void FreezeMapToOdom();
    void UnfreezeMapToOdom();

   private:
    bool HasEnoughAccumulationLocked() const;
    domain::sensor::ScanSnapshot BuildAccumulatedSnapshotLocked(
        const domain::sensor::ScanSnapshot& latest) const;
    domain::result::LocalizationResult ToLocalizationResult(
        const domain::result::AlignmentResult& alignment,
        const domain::sensor::ScanSnapshot& snapshot) const;
    domain::geometry::Pose3 ResolveOdomPoseHint(const domain::sensor::ScanSnapshot& snapshot) const;
    void TransitionToLocked(domain::result::RelocalizationState state);

    Options options_;
    std::shared_ptr<domain::contracts::IGlobalInitializer> global_initializer_ = nullptr;
    std::shared_ptr<domain::contracts::ILocalTracker> local_tracker_ = nullptr;
    std::shared_ptr<domain::contracts::IMapOdomAuthority> map_odom_authority_ = nullptr;
    std::shared_ptr<domain::contracts::IEventSink> event_sink_ = nullptr;

    mutable std::mutex mutex_;
    domain::result::RelocalizationState state_ = domain::result::RelocalizationState::kIdle;
    domain::geometry::Pose3 last_tracking_pose_ = domain::geometry::Pose3::Identity();
    domain::result::LocalizationResult latest_localization_result_;
    std::vector<domain::sensor::CloudData> accumulated_scans_;
};

}  // namespace lightning::application::system
