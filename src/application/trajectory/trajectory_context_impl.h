#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "common/point_def.h"
#include "domain/contracts/localizer.h"
#include "domain/contracts/motion_estimator.h"
#include "domain/contracts/pose_graph_backend.h"
#include "domain/contracts/sensor_collator.h"
#include "domain/contracts/sensor_pipeline.h"
#include "domain/contracts/state_estimator.h"
#include "domain/contracts/trajectory_context.h"
#include "domain/result/motion_estimate.h"
#include "domain/sensor/scan_snapshot.h"

namespace lightning::application::system {
struct LocalizationAssembly;
class RelocalizationCoordinator;
}

namespace lightning::application::trajectory {

class TrajectoryContextImpl : public domain::contracts::ITrajectoryContext {
   public:
    struct Options {
        std::string id;
        std::string config_path;
        bool online_mode = false;
    };

    TrajectoryContextImpl(Options options, system::LocalizationAssembly assembly);
    ~TrajectoryContextImpl() override;

    bool Initialize();

    std::string GetId() const override;
    void Start() override;
    void Stop() override;
    void FeedImu(const domain::sensor::ImuData& imu) override;
    void FeedCloud(const domain::sensor::CloudData& cloud) override;
    void SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) override;
    domain::result::StateEstimate GetLatestStateEstimate() const override;
    domain::result::LocalizationResult GetLatestLocalizationResult() const override;

    void SetInitialPose(const domain::geometry::Pose3& pose) override;

   private:
    void WireTrajectoryFlow();
    void ConfigureLocalizationWorker();
    bool ProcessKeyframeScanWithCoordinator(const CloudPtr& cloud);
    void ProcessKeyframeScanLegacyFallback(const CloudPtr& cloud);
    domain::sensor::ScanSnapshot BuildScanSnapshotFromKeyframe(const CloudPtr& cloud) const;
    void HandleLocalizationResult(const CloudPtr& cloud,
                                  const domain::result::LocalizationResult& result,
                                  bool publish_event);
    void PublishLocalizationResult(const domain::result::LocalizationResult& result);
    void PublishCloudInWorld(const CloudPtr& cloud, const domain::result::LocalizationResult& result);

    struct LegacyRuntimeResources;
    std::unique_ptr<LegacyRuntimeResources> legacy_;

    Options options_;
    std::shared_ptr<domain::contracts::ISensorCollator> sensor_collator_ = nullptr;
    std::shared_ptr<domain::contracts::ISensorPipeline> sensor_pipeline_ = nullptr;
    std::shared_ptr<domain::contracts::IMotionEstimator> motion_estimator_ = nullptr;
    std::shared_ptr<domain::contracts::ILocalizer> localizer_ = nullptr;
    std::shared_ptr<domain::contracts::IStateEstimator> state_estimator_ = nullptr;
    std::shared_ptr<domain::contracts::IPoseGraphBackend> pose_graph_backend_ = nullptr;
    std::shared_ptr<system::RelocalizationCoordinator> relocalization_coordinator_ = nullptr;
    std::shared_ptr<domain::contracts::IEventSink> event_sink_ = nullptr;

    mutable std::mutex mutex_;
    bool initialized_ = false;
    bool started_ = false;
    bool enable_lidar_loc_skip_ = true;
    int lidar_loc_skip_num_ = 1;

    domain::result::StateEstimate latest_state_estimate_;
    domain::result::MotionEstimate latest_motion_estimate_;
    domain::result::LocalizationResult latest_localization_result_;
};

}  // namespace lightning::application::trajectory
