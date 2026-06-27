#include "application/trajectory/trajectory_context_impl.h"

#include <cstdint>
#include <utility>

#include <glog/logging.h>

#include "application/system/system_assembler.h"
#include "application/system/relocalization_coordinator.h"
#include "application/trajectory/trajectory_legacy_conversion.h"
#include "common/eigen_types.h"
#include "common/point_def.h"
#include "common/std_types.h"
#include "core/system/async_message_process.h"
#include "interfaces/fusion_engine.h"
#include "interfaces/localizer.h"
#include "interfaces/sensor_pipeline.h"
#include "io/yaml_io.h"

namespace lightning::application::trajectory {

struct TrajectoryContextImpl::LegacyRuntimeResources {
    explicit LegacyRuntimeResources(std::shared_ptr<loc::IFusionEngine> fusion_engine)
        : legacy_fusion_engine(std::move(fusion_engine)) {}

    sys::AsyncMessageProcess<CloudPtr> localization_proc_cloud;
    std::shared_ptr<loc::IFusionEngine> legacy_fusion_engine = nullptr;
};

TrajectoryContextImpl::TrajectoryContextImpl(Options options, system::LocalizationAssembly assembly)
    : legacy_(std::make_unique<LegacyRuntimeResources>(std::move(assembly.legacy_fusion_engine))),
      options_(std::move(options)),
      sensor_collator_(std::move(assembly.sensor_collator)),
      sensor_pipeline_(std::move(assembly.sensor_pipeline)),
      motion_estimator_(std::move(assembly.motion_estimator)),
      localizer_(std::move(assembly.localizer)),
      state_estimator_(std::move(assembly.state_estimator)),
      pose_graph_backend_(std::move(assembly.pose_graph_backend)),
      relocalization_coordinator_(std::move(assembly.relocalization_coordinator)) {
    WireTrajectoryFlow();
}

TrajectoryContextImpl::~TrajectoryContextImpl() { Stop(); }

bool TrajectoryContextImpl::Initialize() {
    UL lock(mutex_);
    if (initialized_) {
        return true;
    }

    if (!sensor_collator_ || !sensor_pipeline_ || !motion_estimator_ || !localizer_ || !state_estimator_) {
        LOG(ERROR) << "trajectory context assembly is incomplete for trajectory: " << options_.id;
        return false;
    }

    YAML_IO yaml(options_.config_path);
    if (!yaml.IsOpened()) {
        LOG(ERROR) << "failed to open config for trajectory: " << options_.config_path;
        return false;
    }

    enable_lidar_loc_skip_ = yaml.GetValue<bool>("system", "enable_lidar_loc_skip");
    lidar_loc_skip_num_ = yaml.GetValue<int>("system", "lidar_loc_skip_num");

    ConfigureLocalizationWorker();

    if (!localizer_->Init(options_.config_path)) {
        LOG(ERROR) << "failed to init localizer for trajectory: " << options_.id;
        return false;
    }

    initialized_ = true;
    return true;
}

std::string TrajectoryContextImpl::GetId() const { return options_.id; }

void TrajectoryContextImpl::Start() {
    if (!Initialize()) {
        return;
    }

    UL lock(mutex_);
    if (started_) {
        return;
    }

    sensor_collator_->Start();
    if (options_.online_mode) {
        legacy_->localization_proc_cloud.Start();
        sensor_pipeline_->Start();
    }
    started_ = true;
}

void TrajectoryContextImpl::Stop() {
    UL lock(mutex_);
    if (!initialized_ && !started_) {
        return;
    }

    if (sensor_collator_) {
        sensor_collator_->Stop();
        sensor_collator_->Reset();
    }
    if (sensor_pipeline_) {
        sensor_pipeline_->Finish();
    }
    if (localizer_) {
        localizer_->Finish();
    }
    if (legacy_) {
        legacy_->localization_proc_cloud.Quit();
    }

    started_ = false;
}

void TrajectoryContextImpl::FeedImu(const domain::sensor::ImuData& imu) {
    if (sensor_collator_) {
        sensor_collator_->AddImuMeasurement(imu);
    }
}

void TrajectoryContextImpl::FeedCloud(const domain::sensor::CloudData& cloud) {
    if (sensor_collator_) {
        sensor_collator_->AddCloudMeasurement(cloud);
    }
}

void TrajectoryContextImpl::SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) {
    UL lock(mutex_);
    event_sink_ = std::move(sink);
    if (relocalization_coordinator_) {
        relocalization_coordinator_->SetEventSink(event_sink_);
    }
}

domain::result::StateEstimate TrajectoryContextImpl::GetLatestStateEstimate() const {
    UL lock(mutex_);
    return latest_state_estimate_;
}

domain::result::LocalizationResult TrajectoryContextImpl::GetLatestLocalizationResult() const {
    UL lock(mutex_);
    return latest_localization_result_;
}

void TrajectoryContextImpl::SetInitialPose(const domain::geometry::Pose3& pose) {
    if (localizer_) {
        localizer_->SetInitialPose(SE3(pose.rotation, pose.translation));
    }
}

bool TrajectoryContextImpl::ProcessKeyframeScanWithCoordinator(const CloudPtr& cloud) {
    if (!relocalization_coordinator_) {
        return false;
    }

    const auto snapshot = BuildScanSnapshotFromKeyframe(cloud);
    const auto alignment = relocalization_coordinator_->ProcessScan(snapshot);
    if (!alignment.success) {
        return true;
    }

    const auto result = relocalization_coordinator_->GetLatestLocalizationResult();
    HandleLocalizationResult(cloud, result, true);
    return true;
}

void TrajectoryContextImpl::ProcessKeyframeScanLegacyFallback(const CloudPtr& cloud) {
    if (!localizer_) {
        return;
    }

    localizer_->ProcessKeyframeScan(cloud);
    const auto legacy_result = localizer_->GetLocalizationResult();
    const auto result = legacy::ToLocalizationResult(legacy_result);
    HandleLocalizationResult(cloud, result, false);
    FeedLocalizationToBackends(&legacy_result, result, true);
}

domain::sensor::ScanSnapshot TrajectoryContextImpl::BuildScanSnapshotFromKeyframe(const CloudPtr& cloud) const {
    domain::sensor::ScanSnapshot snapshot;
    snapshot.stamp_ns = cloud ? static_cast<std::uint64_t>(cloud->header.stamp) : 0;
    snapshot.frame_id = options_.id + "/lidar";
    snapshot.source_id = options_.id;
    snapshot.registered_scan = legacy::ToDomainCloud(cloud, snapshot.stamp_ns);
    snapshot.registered_scan.frame_id = snapshot.frame_id;
    snapshot.registered_scan.sensor_id = snapshot.source_id;

    UL lock(mutex_);
    if (latest_state_estimate_.valid) {
        snapshot.odom_pose_hint = latest_state_estimate_.pose;
        snapshot.has_odom_pose_hint = true;
    } else if (latest_motion_estimate_.valid) {
        snapshot.odom_pose_hint = latest_motion_estimate_.pose;
        snapshot.has_odom_pose_hint = true;
    }
    return snapshot;
}

void TrajectoryContextImpl::HandleLocalizationResult(const CloudPtr& cloud,
                                                     const domain::result::LocalizationResult& result,
                                                     bool publish_event) {
    {
        UL lock(mutex_);
        latest_localization_result_ = result;
    }

    if (state_estimator_) {
        state_estimator_->FeedLocalizationResult(result);
    }

    PublishCloudInWorld(cloud, result);

    if (publish_event) {
        PublishLocalizationResult(result);
    }
}

void TrajectoryContextImpl::FeedMotionEstimateToBackends(
    const NavState& legacy_state,
    const domain::result::MotionEstimate& estimate) {
    if (legacy_ && legacy_->legacy_fusion_engine) {
        switch (estimate.source) {
            case domain::result::MotionEstimateSource::kDeadReckoning:
                legacy_->legacy_fusion_engine->FeedDeadReckoning(legacy_state);
                break;
            case domain::result::MotionEstimateSource::kLidarOdometry:
                legacy_->legacy_fusion_engine->FeedLidarOdom(legacy_state);
                break;
            case domain::result::MotionEstimateSource::kUnknown:
            case domain::result::MotionEstimateSource::kExternalPrior:
                break;
        }
    }

    if (pose_graph_backend_) {
        pose_graph_backend_->FeedMotionEstimate(estimate);
    }
}

void TrajectoryContextImpl::FeedLocalizationToBackends(
    const loc::LocalizationResult* legacy_result,
    const domain::result::LocalizationResult& result,
    bool publish_if_unhandled) {
    if (legacy_ && legacy_->legacy_fusion_engine && legacy_result) {
        legacy_->legacy_fusion_engine->FeedLocalization(*legacy_result);
    }

    if (pose_graph_backend_) {
        pose_graph_backend_->FeedLocalizationResult(result);
    }

    if (publish_if_unhandled && !pose_graph_backend_) {
        PublishLocalizationResult(result);
    }
}

void TrajectoryContextImpl::PublishLocalizationResult(const domain::result::LocalizationResult& result) {
    std::shared_ptr<domain::contracts::IEventSink> sink;
    {
        UL lock(mutex_);
        sink = event_sink_;
    }
    if (sink) {
        sink->OnLocalizationResult(result);
    }
}

void TrajectoryContextImpl::WireTrajectoryFlow() {
    if (sensor_collator_) {
        sensor_collator_->SetImuHandler([this](const domain::sensor::ImuData& imu) {
            if (sensor_pipeline_) {
                sensor_pipeline_->ProcessIMU(legacy::ToLegacyImu(imu));
            }
        });

        sensor_collator_->SetCloudHandler([this](const domain::sensor::CloudData& cloud) {
            if (sensor_pipeline_) {
                sensor_pipeline_->ProcessCloud(legacy::ToLegacyCloud(cloud));
            }
        });
    }

    if (sensor_pipeline_) {
        sensor_pipeline_->SetDeadReckoningCallback([this](const NavState& state) {
            auto estimate = legacy::ToMotionEstimate(state, domain::result::MotionEstimateSource::kDeadReckoning);

            {
                UL lock(mutex_);
                latest_motion_estimate_ = estimate;
            }

            if (localizer_) {
                localizer_->FeedDeadReckoning(state);
            }
            if (state_estimator_) {
                state_estimator_->FeedMotionEstimate(estimate);
            }
            FeedMotionEstimateToBackends(state, estimate);

            std::shared_ptr<domain::contracts::IEventSink> sink;
            {
                UL lock(mutex_);
                sink = event_sink_;
            }
            if (sink) {
                sink->OnMotionEstimate(estimate);
            }
        });

        sensor_pipeline_->SetLidarOdomCallback([this](const NavState& state) {
            auto estimate = legacy::ToMotionEstimate(state, domain::result::MotionEstimateSource::kLidarOdometry);

            {
                UL lock(mutex_);
                latest_motion_estimate_ = estimate;
            }

            if (localizer_) {
                localizer_->FeedLidarOdom(state);
            }
            if (state_estimator_) {
                state_estimator_->FeedMotionEstimate(estimate);
            }
            FeedMotionEstimateToBackends(state, estimate);

            std::shared_ptr<domain::contracts::IEventSink> sink;
            {
                UL lock(mutex_);
                sink = event_sink_;
            }
            if (sink) {
                sink->OnMotionEstimate(estimate);
            }
        });

        sensor_pipeline_->SetKeyframeScanCallback([this](CloudPtr cloud) {
            if (options_.online_mode) {
                legacy_->localization_proc_cloud.AddMessage(cloud);
                return;
            }

            if (!ProcessKeyframeScanWithCoordinator(cloud)) {
                ProcessKeyframeScanLegacyFallback(cloud);
            }
        });
    }

    if (state_estimator_) {
        state_estimator_->SetOutputCallback([this](const domain::result::StateEstimate& estimate) {
            std::shared_ptr<domain::contracts::IEventSink> sink;
            {
                UL lock(mutex_);
                latest_state_estimate_ = estimate;
                sink = event_sink_;
            }
            if (sink) {
                sink->OnStateEstimate(estimate);
            }
        });
    }

    if (pose_graph_backend_) {
        pose_graph_backend_->SetOutputCallback([this](const domain::result::LocalizationResult& result) {
            std::shared_ptr<domain::contracts::IEventSink> sink;
            {
                UL lock(mutex_);
                latest_localization_result_ = result;
                sink = event_sink_;
            }
            if (sink) {
                sink->OnLocalizationResult(result);
            }
        });
    }
}

void TrajectoryContextImpl::ConfigureLocalizationWorker() {
    if (!legacy_) {
        return;
    }

    legacy_->localization_proc_cloud.SetName(options_.id + "/lidar_loc");
    legacy_->localization_proc_cloud.SetMaxSize(1);
    legacy_->localization_proc_cloud.SetSkipParam(enable_lidar_loc_skip_, lidar_loc_skip_num_);
    legacy_->localization_proc_cloud.SetProcFunc([this](const CloudPtr& cloud) {
        if (!ProcessKeyframeScanWithCoordinator(cloud)) {
            ProcessKeyframeScanLegacyFallback(cloud);
        }
    });
}

void TrajectoryContextImpl::PublishCloudInWorld(const CloudPtr& cloud,
                                                const domain::result::LocalizationResult& result) {
    if (cloud == nullptr || !result.localizer_valid) {
        return;
    }

    std::shared_ptr<domain::contracts::IEventSink> sink;
    {
        UL lock(mutex_);
        sink = event_sink_;
    }
    if (!sink) {
        return;
    }

    const auto stamp_ns = static_cast<std::uint64_t>(result.timestamp_s * 1e9);
    sink->OnCloudInWorld(legacy::ToDomainCloud(cloud, stamp_ns), result.pose);
}

}  // namespace lightning::application::trajectory
