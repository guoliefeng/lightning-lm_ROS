#include "application/trajectory/trajectory_context_impl.h"

#include <utility>

#include <glog/logging.h>

#include "core/localization/localization_result.h"
#include "interfaces/localizer.h"
#include "interfaces/sensor_pipeline.h"
#include "io/yaml_io.h"

namespace lightning::application::trajectory {

namespace {

domain::geometry::Pose3 ToPose3(const SE3& pose) {
    domain::geometry::Pose3 converted;
    converted.translation = pose.translation();
    converted.rotation = pose.unit_quaternion();
    return converted;
}

domain::result::MotionEstimate ToMotionEstimate(const NavState& state, domain::result::MotionEstimateSource source) {
    domain::result::MotionEstimate estimate;
    estimate.timestamp_s = state.timestamp_;
    estimate.pose = ToPose3(state.GetPose());
    estimate.linear_velocity = state.GetVel();
    estimate.angular_velocity = state.Getbg();
    estimate.confidence = state.confidence_;
    estimate.valid = state.pose_is_ok_;
    estimate.stationary = state.is_parking_;
    estimate.source = source;
    return estimate;
}

domain::result::LocalizationStatus ToDomainStatus(loc::LocalizationStatus status) {
    switch (status) {
        case loc::LocalizationStatus::IDLE:
            return domain::result::LocalizationStatus::kIdle;
        case loc::LocalizationStatus::INITIALIZING:
            return domain::result::LocalizationStatus::kInitializing;
        case loc::LocalizationStatus::GOOD:
            return domain::result::LocalizationStatus::kGood;
        case loc::LocalizationStatus::FOLLOWING_DR:
            return domain::result::LocalizationStatus::kFollowingMotion;
        case loc::LocalizationStatus::FAIL:
            return domain::result::LocalizationStatus::kFail;
    }
    return domain::result::LocalizationStatus::kFail;
}

domain::result::LocalizationResult ToLocalizationResult(const loc::LocalizationResult& result) {
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

IMUPtr ToLegacyImu(const domain::sensor::ImuData& imu) {
    auto converted = std::make_shared<IMU>();
    converted->timestamp = static_cast<double>(imu.stamp_ns) * 1e-9;
    converted->angular_velocity = imu.angular_velocity;
    converted->linear_acceleration = imu.linear_acceleration;
    return converted;
}

domain::sensor::ImuData ToDomainImu(const IMUPtr& imu) {
    domain::sensor::ImuData converted;
    if (imu == nullptr) {
        return converted;
    }
    converted.stamp_ns = static_cast<std::uint64_t>(imu->timestamp * 1e9);
    converted.angular_velocity = imu->angular_velocity;
    converted.linear_acceleration = imu->linear_acceleration;
    return converted;
}

SensorCloudInput ToLegacyCloud(const domain::sensor::CloudData& cloud) {
    SensorCloudInput converted;
    converted.stamp_ns = cloud.stamp_ns;
    converted.cloud.reset(new PointCloudType());
    converted.cloud->reserve(cloud.points.size());
    for (const auto& point : cloud.points) {
        PointType pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        pt.intensity = point.intensity;
        pt.time = point.relative_time_s;
        converted.cloud->push_back(pt);
    }
    return converted;
}

}  // namespace

TrajectoryContextImpl::TrajectoryContextImpl(Options options, system::LocalizationAssembly assembly)
    : options_(std::move(options)),
      sensor_collator_(std::move(assembly.sensor_collator)),
      sensor_pipeline_(std::move(assembly.sensor_pipeline)),
      motion_estimator_(std::move(assembly.motion_estimator)),
      localizer_(std::move(assembly.localizer)),
      state_estimator_(std::move(assembly.state_estimator)),
      pose_graph_backend_(std::move(assembly.pose_graph_backend)),
      legacy_fusion_engine_(std::move(assembly.legacy_fusion_engine)) {
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
        localization_proc_cloud_.Start();
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
    localization_proc_cloud_.Quit();
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
}

domain::result::StateEstimate TrajectoryContextImpl::GetLatestStateEstimate() const {
    UL lock(mutex_);
    return latest_state_estimate_;
}

domain::result::LocalizationResult TrajectoryContextImpl::GetLatestLocalizationResult() const {
    UL lock(mutex_);
    return latest_localization_result_;
}

void TrajectoryContextImpl::FeedLegacyImu(const IMUPtr& imu) { FeedImu(ToDomainImu(imu)); }

void TrajectoryContextImpl::FeedLegacyCloud(const SensorCloudInput& cloud) {
    if (!sensor_pipeline_) {
        return;
    }
    sensor_pipeline_->ProcessCloud(cloud);
}

void TrajectoryContextImpl::SetInitialPose(const SE3& pose) {
    if (localizer_) {
        localizer_->SetInitialPose(pose);
    }
}

void TrajectoryContextImpl::WireTrajectoryFlow() {
    if (sensor_collator_) {
        sensor_collator_->SetImuHandler([this](const domain::sensor::ImuData& imu) { HandleCollatedImu(imu); });
        sensor_collator_->SetCloudHandler([this](const domain::sensor::CloudData& cloud) { HandleCollatedCloud(cloud); });
    }

    if (sensor_pipeline_) {
        sensor_pipeline_->SetDeadReckoningCallback([this](const NavState& state) { HandleDeadReckoning(state); });
        sensor_pipeline_->SetLidarOdomCallback([this](const NavState& state) { HandleLidarOdometry(state); });
        sensor_pipeline_->SetKeyframeScanCallback([this](CloudPtr cloud) { HandleKeyframeCloud(cloud); });
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
    localization_proc_cloud_.SetName(options_.id + "/lidar_loc");
    localization_proc_cloud_.SetMaxSize(1);
    localization_proc_cloud_.SetSkipParam(enable_lidar_loc_skip_, lidar_loc_skip_num_);
    localization_proc_cloud_.SetProcFunc([this](const CloudPtr& cloud) { ProcessLocalizationCloud(cloud); });
}

void TrajectoryContextImpl::HandleCollatedImu(const domain::sensor::ImuData& imu) {
    if (sensor_pipeline_) {
        sensor_pipeline_->ProcessIMU(ToLegacyImu(imu));
    }
}

void TrajectoryContextImpl::HandleCollatedCloud(const domain::sensor::CloudData& cloud) {
    if (sensor_pipeline_) {
        sensor_pipeline_->ProcessCloud(ToLegacyCloud(cloud));
    }
}

void TrajectoryContextImpl::HandleDeadReckoning(const NavState& state) {
    auto estimate = ToMotionEstimate(state, domain::result::MotionEstimateSource::kDeadReckoning);

    if (localizer_) {
        localizer_->FeedDeadReckoning(state);
    }
    if (state_estimator_) {
        state_estimator_->FeedMotionEstimate(estimate);
    }
    if (legacy_fusion_engine_) {
        legacy_fusion_engine_->FeedDeadReckoning(state);
    }

    std::shared_ptr<domain::contracts::IEventSink> sink;
    {
        UL lock(mutex_);
        sink = event_sink_;
    }
    if (sink) {
        sink->OnMotionEstimate(estimate);
    }
}

void TrajectoryContextImpl::HandleLidarOdometry(const NavState& state) {
    auto estimate = ToMotionEstimate(state, domain::result::MotionEstimateSource::kLidarOdometry);

    if (localizer_) {
        localizer_->FeedLidarOdom(state);
    }
    if (state_estimator_) {
        state_estimator_->FeedMotionEstimate(estimate);
    }
    if (legacy_fusion_engine_) {
        legacy_fusion_engine_->FeedLidarOdom(state);
    } else if (pose_graph_backend_) {
        pose_graph_backend_->FeedMotionEstimate(estimate);
    }

    std::shared_ptr<domain::contracts::IEventSink> sink;
    {
        UL lock(mutex_);
        sink = event_sink_;
    }
    if (sink) {
        sink->OnMotionEstimate(estimate);
    }
}

void TrajectoryContextImpl::HandleKeyframeCloud(const CloudPtr& cloud) {
    if (options_.online_mode) {
        localization_proc_cloud_.AddMessage(cloud);
    } else {
        ProcessLocalizationCloud(cloud);
    }
}

void TrajectoryContextImpl::ProcessLocalizationCloud(const CloudPtr& cloud) {
    if (!localizer_) {
        return;
    }

    localizer_->ProcessKeyframeScan(cloud);
    const auto legacy_result = localizer_->GetLocalizationResult();
    const auto result = ToLocalizationResult(legacy_result);

    {
        UL lock(mutex_);
        latest_localization_result_ = result;
    }

    if (state_estimator_) {
        state_estimator_->FeedLocalizationResult(result);
    }

    if (legacy_fusion_engine_) {
        legacy_fusion_engine_->FeedLocalization(legacy_result);
    } else if (pose_graph_backend_) {
        pose_graph_backend_->FeedLocalizationResult(result);
    } else {
        std::shared_ptr<domain::contracts::IEventSink> sink;
        {
            UL lock(mutex_);
            sink = event_sink_;
        }
        if (sink) {
            sink->OnLocalizationResult(result);
        }
    }
}

}  // namespace lightning::application::trajectory
