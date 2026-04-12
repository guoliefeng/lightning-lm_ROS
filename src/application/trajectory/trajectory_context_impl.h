#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "application/system/system_assembler.h"
#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/point_def.h"
#include "common/sensor_cloud_input.h"
#include "common/std_types.h"
#include "core/system/async_message_process.h"
#include "domain/contracts/event_sink.h"
#include "domain/contracts/trajectory_context.h"
#include "domain/result/localization_result.h"
#include "domain/result/state_estimate.h"
#include "interfaces/fusion_engine.h"

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

    void FeedLegacyImu(const IMUPtr& imu);
    void FeedLegacyCloud(const SensorCloudInput& cloud);
    void SetInitialPose(const SE3& pose);

   private:
    void WireTrajectoryFlow();
    void ConfigureLocalizationWorker();
    void HandleCollatedImu(const domain::sensor::ImuData& imu);
    void HandleCollatedCloud(const domain::sensor::CloudData& cloud);
    void HandleDeadReckoning(const NavState& state);
    void HandleLidarOdometry(const NavState& state);
    void HandleKeyframeCloud(const CloudPtr& cloud);
    void ProcessLocalizationCloud(const CloudPtr& cloud);

    Options options_;
    std::shared_ptr<domain::contracts::ISensorCollator> sensor_collator_ = nullptr;
    std::shared_ptr<domain::contracts::ISensorPipeline> sensor_pipeline_ = nullptr;
    std::shared_ptr<domain::contracts::IMotionEstimator> motion_estimator_ = nullptr;
    std::shared_ptr<domain::contracts::ILocalizer> localizer_ = nullptr;
    std::shared_ptr<domain::contracts::IStateEstimator> state_estimator_ = nullptr;
    std::shared_ptr<domain::contracts::IPoseGraphBackend> pose_graph_backend_ = nullptr;
    std::shared_ptr<domain::contracts::IEventSink> event_sink_ = nullptr;

    std::shared_ptr<loc::IFusionEngine> legacy_fusion_engine_ = nullptr;

    mutable std::mutex mutex_;
    bool initialized_ = false;
    bool started_ = false;
    bool enable_lidar_loc_skip_ = true;
    int lidar_loc_skip_num_ = 1;

    sys::AsyncMessageProcess<CloudPtr> localization_proc_cloud_;

    domain::result::StateEstimate latest_state_estimate_;
    domain::result::LocalizationResult latest_localization_result_;
};

}  // namespace lightning::application::trajectory
