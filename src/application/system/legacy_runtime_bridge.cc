#include "application/system/legacy_runtime_bridge.h"

#include <utility>

#include <glog/logging.h>

#include "application/trajectory/trajectory_legacy_conversion.h"
#include "domain/contracts/event_sink.h"
#include "domain/contracts/system_root.h"
#include "domain/result/map_state.h"
#include "domain/result/motion_estimate.h"
#include "domain/result/state_estimate.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"

namespace lightning::application::system {

namespace {

loc::LocalizationStatus ToLegacyStatus(domain::result::LocalizationStatus status) {
    switch (status) {
        case domain::result::LocalizationStatus::kIdle:
            return loc::LocalizationStatus::IDLE;
        case domain::result::LocalizationStatus::kInitializing:
            return loc::LocalizationStatus::INITIALIZING;
        case domain::result::LocalizationStatus::kGood:
            return loc::LocalizationStatus::GOOD;
        case domain::result::LocalizationStatus::kFollowingMotion:
            return loc::LocalizationStatus::FOLLOWING_DR;
        case domain::result::LocalizationStatus::kFail:
            return loc::LocalizationStatus::FAIL;
    }
    return loc::LocalizationStatus::FAIL;
}

loc::LocalizationResult ToLegacyLocalizationResult(const domain::result::LocalizationResult& result) {
    loc::LocalizationResult converted;
    converted.timestamp_ = result.timestamp_s;
    converted.pose_ = SE3(result.pose.rotation, result.pose.translation);
    converted.valid_ = result.valid;
    converted.status_ = ToLegacyStatus(result.status);
    converted.lidar_loc_valid_ = result.localizer_valid;
    converted.lidar_loc_inlier_ = result.inlier;
    converted.lidar_loc_odom_error_normal_ = result.motion_consistent;
    converted.lidar_loc_smooth_flag_ = result.smooth;
    converted.confidence_ = result.confidence;
    converted.lidar_loc_odom_delta_ = result.motion_delta_m;
    return converted;
}

}  // namespace

class LegacyRuntimeBridge::BridgeEventSink : public domain::contracts::IEventSink {
   public:
    explicit BridgeEventSink(LegacyRuntimeBridge* bridge) : bridge_(bridge) {}

    void OnMotionEstimate(const domain::result::MotionEstimate&) override {}

    void OnStateEstimate(const domain::result::StateEstimate& estimate) override {
        if (bridge_) {
            bridge_->HandleStateEstimate(estimate);
        }
    }

    void OnLocalizationResult(const domain::result::LocalizationResult& result) override {
        if (bridge_) {
            bridge_->HandleLocalizationResult(result);
        }
    }

    void OnRelocalizationState(domain::result::RelocalizationState) override {}

    void OnMapState(const domain::result::MapState&) override {}

    void OnCloudInWorld(const domain::sensor::CloudData& cloud, const domain::geometry::Pose3& pose) override {
        if (bridge_) {
            bridge_->HandleCloudInWorld(cloud, pose);
        }
    }

   private:
    LegacyRuntimeBridge* bridge_ = nullptr;
};

LegacyRuntimeBridge::LegacyRuntimeBridge(std::shared_ptr<domain::contracts::ISystemRoot> system_root,
                                         std::string trajectory_id)
    : system_root_(std::move(system_root)), trajectory_id_(std::move(trajectory_id)) {}

LegacyRuntimeBridge::~LegacyRuntimeBridge() { Finish(); }

bool LegacyRuntimeBridge::Init(const std::string& yaml_path, std::shared_ptr<domain::contracts::ISystemRoot> root) {
    system_root_ = std::move(root);
    if (!system_root_) {
        LOG(ERROR) << "LegacyRuntimeBridge requires an ISystemRoot";
        return false;
    }
    return Init(yaml_path);
}

bool LegacyRuntimeBridge::Init(const std::string& yaml_path) {
    auto attached_ui = owns_ui_ ? nullptr : ui_;
    Finish();
    if (attached_ui) {
        ui_ = std::move(attached_ui);
    }

    if (!system_root_) {
        LOG(ERROR) << "LegacyRuntimeBridge requires an ISystemRoot";
        return false;
    }

    YAML_IO yaml(yaml_path);
    const bool with_ui = yaml.GetValue<bool>("system", "with_ui");

    cloud_converter_ = std::make_unique<trajectory::legacy::LegacyCloudConverter>(yaml_path);
    if (!cloud_converter_->IsValid()) {
        return false;
    }

    if (with_ui && !ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        owns_ui_ = true;
        ui_->SetCurrentScanSize(1);
        ui_->Init();
    }

    CreateEventSink();
    shutdown_needed_ = true;
    if (!system_root_->Init(yaml_path)) {
        return false;
    }

    system_root_->SetEventSink(trajectory_id_, event_sink_);
    if (!system_root_->Start()) {
        return false;
    }
    initialized_ = true;
    return true;
}

void LegacyRuntimeBridge::Finish() {
    if (system_root_ && shutdown_needed_) {
        shutdown_needed_ = false;
        system_root_->Shutdown();
    }

    event_sink_.reset();
    cloud_converter_.reset();

    if (ui_ && owns_ui_) {
        ui_->Quit();
    }
    ui_.reset();
    owns_ui_ = false;

    latest_localization_result_ = loc::LocalizationResult();
    last_imu_time_ = 0.0;
    last_cloud_time_ = 0.0;
    initialized_ = false;
}

void LegacyRuntimeBridge::FeedLegacyImu(const IMUPtr& imu) {
    if (!initialized_ || !system_root_ || imu == nullptr) {
        return;
    }

    const double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    system_root_->FeedImu(trajectory_id_, trajectory::legacy::ToDomainImu(imu));
}

void LegacyRuntimeBridge::FeedLegacyCloud(const SensorCloudInput& cloud) {
    if (!initialized_ || !system_root_ || !cloud_converter_) {
        return;
    }

    const double this_cloud_time = static_cast<double>(cloud.stamp_ns) * 1e-9;
    if (last_cloud_time_ > 0 && this_cloud_time < last_cloud_time_) {
        LOG(WARNING) << "Cloud 时间异常：" << this_cloud_time << ", last: " << last_cloud_time_;
    }
    last_cloud_time_ = this_cloud_time;

    auto domain_cloud = cloud_converter_->ToDomainCloud(cloud);
    if (domain_cloud.points.empty()) {
        return;
    }
    system_root_->FeedCloud(trajectory_id_, domain_cloud);
}

void LegacyRuntimeBridge::SetInitialPose(const SE3& pose) {
    if (!initialized_ || !system_root_) {
        return;
    }
    system_root_->SetInitialPose(trajectory_id_, trajectory::legacy::ToPose3(pose));
}

void LegacyRuntimeBridge::SetTFCallback(TFCallback callback) { tf_callback_ = std::move(callback); }

void LegacyRuntimeBridge::SetLocStateCallback(LocStateCallback callback) {
    loc_state_callback_ = std::move(callback);
}

void LegacyRuntimeBridge::SetLegacyOutputHandlers(TFCallback tf_callback, LocStateCallback loc_state_callback) {
    tf_callback_ = std::move(tf_callback);
    loc_state_callback_ = std::move(loc_state_callback);
}

void LegacyRuntimeBridge::AttachUi(std::shared_ptr<ui::PangolinWindow> ui) {
    if (ui_ && owns_ui_) {
        ui_->Quit();
    }
    ui_ = std::move(ui);
    owns_ui_ = false;
}

std::shared_ptr<domain::contracts::IEventSink> LegacyRuntimeBridge::CreateEventSink() {
    if (!event_sink_) {
        event_sink_ = std::make_shared<BridgeEventSink>(this);
    }
    return event_sink_;
}

void LegacyRuntimeBridge::HandleLocalizationResult(const domain::result::LocalizationResult& result) {
    if (latest_localization_result_.timestamp_ > 0) {
        const double dt = result.timestamp_s - latest_localization_result_.timestamp_;
        if (dt > 0) {
            LOG_EVERY_N(INFO, 10) << "loc fps: " << 1.0 / dt;
        }
    }

    latest_localization_result_ = ToLegacyLocalizationResult(result);

    if (tf_callback_ && latest_localization_result_.valid_) {
        tf_callback_(latest_localization_result_.ToGeoMsg());
    }

    if (ui_) {
        ui_->UpdateNavState(latest_localization_result_.ToNavState());
        ui_->UpdateRecentPose(latest_localization_result_.pose_);
    }

    if (loc_state_callback_) {
        std_msgs::Int32 loc_state;
        loc_state.data = static_cast<int>(result.status);
        LOG(INFO) << "loc_state: " << loc_state.data;
        loc_state_callback_(loc_state);
    }
}

void LegacyRuntimeBridge::HandleStateEstimate(const domain::result::StateEstimate&) {}

void LegacyRuntimeBridge::HandleCloudInWorld(const domain::sensor::CloudData& cloud,
                                             const domain::geometry::Pose3& pose) {
    if (!ui_) {
        return;
    }

    auto legacy_cloud = trajectory::legacy::ToLegacyCloud(cloud);
    if (legacy_cloud.cloud == nullptr || legacy_cloud.cloud->empty()) {
        return;
    }
    ui_->UpdateScan(legacy_cloud.cloud, SE3(pose.rotation, pose.translation));
}

}  // namespace lightning::application::system
