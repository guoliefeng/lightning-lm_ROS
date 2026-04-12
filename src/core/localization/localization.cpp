#include "core/localization/localization.h"

#include "application/system/system_assembler.h"
#include "application/trajectory/trajectory_context_impl.h"
#include "application/trajectory/trajectory_legacy_conversion.h"
#include "application/trajectory/trajectory_manager_impl.h"
#include "core/lightning_math.hpp"
#include "interfaces/localizer.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"

namespace lightning::loc {

namespace {

geometry_msgs::TransformStamped ToGeoMsg(const domain::result::LocalizationResult& result) {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = math::FromSec(result.timestamp_s);
    msg.child_frame_id = "base_link";
    msg.transform.translation.x = result.pose.translation.x();
    msg.transform.translation.y = result.pose.translation.y();
    msg.transform.translation.z = result.pose.translation.z();
    msg.transform.rotation.x = result.pose.rotation.x();
    msg.transform.rotation.y = result.pose.rotation.y();
    msg.transform.rotation.z = result.pose.rotation.z();
    msg.transform.rotation.w = result.pose.rotation.w();
    return msg;
}

LocalizationStatus ToLegacyStatus(domain::result::LocalizationStatus status) {
    switch (status) {
        case domain::result::LocalizationStatus::kIdle:
            return LocalizationStatus::IDLE;
        case domain::result::LocalizationStatus::kInitializing:
            return LocalizationStatus::INITIALIZING;
        case domain::result::LocalizationStatus::kGood:
            return LocalizationStatus::GOOD;
        case domain::result::LocalizationStatus::kFollowingMotion:
            return LocalizationStatus::FOLLOWING_DR;
        case domain::result::LocalizationStatus::kFail:
            return LocalizationStatus::FAIL;
    }
    return LocalizationStatus::FAIL;
}

LocalizationResult ToLegacyLocalizationResult(const domain::result::LocalizationResult& result) {
    LocalizationResult converted;
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

NavState ToNavState(const domain::result::LocalizationResult& result) {
    NavState state;
    state.timestamp_ = result.timestamp_s;
    state.confidence_ = result.confidence;
    state.SetPose(SE3(result.pose.rotation, result.pose.translation));
    state.pose_is_ok_ = result.status == domain::result::LocalizationStatus::kGood;
    return state;
}

class LocalizationRuntimeEventSink : public domain::contracts::IEventSink {
   public:
    using LocalizationHandler = std::function<void(const domain::result::LocalizationResult&)>;
    using StateHandler = std::function<void(const domain::result::StateEstimate&)>;

    LocalizationRuntimeEventSink(LocalizationHandler localization_handler, StateHandler state_handler)
        : localization_handler_(std::move(localization_handler)), state_handler_(std::move(state_handler)) {}

    void OnMotionEstimate(const domain::result::MotionEstimate&) override {}

    void OnStateEstimate(const domain::result::StateEstimate& estimate) override {
        if (state_handler_) {
            state_handler_(estimate);
        }
    }

    void OnLocalizationResult(const domain::result::LocalizationResult& result) override {
        if (localization_handler_) {
            localization_handler_(result);
        }
    }

    void OnMapState(const domain::result::MapState&) override {}

    void OnCloudInWorld(const domain::sensor::CloudData&, const domain::geometry::Pose3&) override {}

   private:
    LocalizationHandler localization_handler_;
    StateHandler state_handler_;
};

}  // namespace

Localization::Localization(Options options) { options_ = options; }

Localization::~Localization() = default;

bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (trajectory_ != nullptr || trajectory_manager_ != nullptr) {
        Finish();
    }

    YAML_IO yaml(yaml_path);
    options_.with_ui_ = yaml.GetValue<bool>("system", "with_ui");
    cloud_converter_ = std::make_unique<application::trajectory::legacy::LegacyCloudConverter>(yaml_path);
    if (!cloud_converter_->IsValid()) {
        return false;
    }

    if (options_.with_ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->SetCurrentScanSize(1);
        ui_->Init();
    }

    application::system::LocalizationAssemblyOptions assembly_options;
    assembly_options.yaml_path = yaml_path;
    assembly_options.global_map_path = global_map_path;
    assembly_options.online_mode = options_.online_mode_;
    assembly_options.with_ui = options_.with_ui_;

    auto create_trajectory = [assembly_options, ui = ui_](const std::string& trajectory_id) {
        auto assembly = application::system::SystemAssembler::AssembleLocalization(assembly_options);
        if (!assembly.localizer || !assembly.sensor_pipeline || !assembly.motion_estimator || !assembly.state_estimator) {
            return std::shared_ptr<application::trajectory::TrajectoryContextImpl>();
        }

        if (ui) {
            assembly.localizer->SetUI(ui);
        }

        application::trajectory::TrajectoryContextImpl::Options context_options;
        context_options.id = trajectory_id;
        context_options.config_path = assembly_options.yaml_path;
        context_options.online_mode = assembly_options.online_mode;
        return std::make_shared<application::trajectory::TrajectoryContextImpl>(context_options, std::move(assembly));
    };

    trajectory_manager_ =
        std::make_shared<application::trajectory::TrajectoryManagerImpl>(std::move(create_trajectory));
    trajectory_ = trajectory_manager_->GetOrCreateTrajectory("default");
    trajectory_impl_ = std::dynamic_pointer_cast<application::trajectory::TrajectoryContextImpl>(trajectory_);
    if (!trajectory_impl_) {
        return false;
    }

    auto event_sink = std::make_shared<LocalizationRuntimeEventSink>(
        [this](const domain::result::LocalizationResult& result) {
            if (loc_result_.timestamp_ > 0) {
                double loc_fps = 1.0 / (result.timestamp_s - loc_result_.timestamp_);
                LOG_EVERY_N(INFO, 10) << "loc fps: " << loc_fps;
            }

            loc_result_ = ToLegacyLocalizationResult(result);

            if (tf_callback_ && loc_result_.valid_) {
                tf_callback_(ToGeoMsg(result));
            }

            if (ui_) {
                ui_->UpdateNavState(ToNavState(result));
                ui_->UpdateRecentPose(loc_result_.pose_);
            }

            if (loc_state_callback_) {
                std_msgs::Int32 loc_state;
                loc_state.data = static_cast<int>(result.status);
                LOG(INFO) << "loc_state: " << loc_state.data;
                loc_state_callback_(loc_state);
            }
        },
        [](const domain::result::StateEstimate&) {});

    trajectory_->SetEventSink(event_sink);
    if (!trajectory_impl_->Initialize()) {
        return false;
    }
    trajectory_->Start();
    return true;
}

void Localization::ProcessCloud(const SensorCloudInput& cloud) {
    UL lock(global_mutex_);
    if (!trajectory_impl_) {
        return;
    }

    double this_cloud_time = static_cast<double>(cloud.stamp_ns) * 1e-9;
    if (last_cloud_time_ > 0 && this_cloud_time < last_cloud_time_) {
        LOG(WARNING) << "Cloud 时间异常：" << this_cloud_time << ", last: " << last_cloud_time_;
    }
    last_cloud_time_ = this_cloud_time;

    auto domain_cloud = cloud_converter_->ToDomainCloud(cloud);
    if (domain_cloud.points.empty()) {
        return;
    }
    trajectory_->FeedCloud(domain_cloud);
}

void Localization::ProcessIMUMsg(IMUPtr imu) {
    UL lock(global_mutex_);
    if (!trajectory_impl_ || imu == nullptr) {
        return;
    }

    double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    trajectory_->FeedImu(application::trajectory::legacy::ToDomainImu(imu));
}

void Localization::Finish() {
    if (trajectory_) {
        trajectory_->Stop();
    }
    if (trajectory_manager_) {
        trajectory_manager_->RemoveTrajectory("default");
    }
    trajectory_impl_.reset();
    trajectory_.reset();
    trajectory_manager_.reset();
    cloud_converter_.reset();

    if (ui_) {
        ui_->Quit();
        ui_.reset();
    }

    loc_result_ = LocalizationResult();
    last_imu_time_ = 0;
    last_odom_time_ = 0;
    last_cloud_time_ = 0;
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    if (trajectory_impl_) {
        domain::geometry::Pose3 pose;
        pose.translation = t;
        pose.rotation = q;
        trajectory_impl_->SetInitialPose(pose);
    }
}

void Localization::SetTFCallback(Localization::TFCallback callback) { tf_callback_ = std::move(callback); }

void Localization::SetLocStateCallback(Localization::LocStateCallback callback) {
    loc_state_callback_ = std::move(callback);
}

}  // namespace lightning::loc
