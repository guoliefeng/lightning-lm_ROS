#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "core/localization/localization.h"
#include "core/localization/localization_builder.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"

namespace lightning::loc {

// ！ 构造函数
Localization::Localization(Options options) { options_ = options; }

// ！初始化函数
bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (localizer_ != nullptr || sensor_pipeline_ != nullptr) {
        // 若已经启动，则变为初始化
        Finish();
    }

    YAML_IO yaml(yaml_path);
    options_.with_ui_ = yaml.GetValue<bool>("system", "with_ui");

    if (options_.with_ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->SetCurrentScanSize(10);
        ui_->Init();
    }

    auto components =
        LocalizationBuilder::BuildLocalizationComponents(yaml_path, global_map_path, options_.with_ui_,
                                                         options_.online_mode_, ui_);
    if (!components.localizer || !components.fusion_engine || !components.sensor_pipeline) {
        return false;
    }

    localizer_ = components.localizer;
    fusion_engine_ = components.fusion_engine;
    sensor_pipeline_ = components.sensor_pipeline;

    sensor_pipeline_->SetDeadReckoningCallback([this](const NavState& dr_state) {
        localizer_->FeedDeadReckoning(dr_state);
        fusion_engine_->FeedDeadReckoning(dr_state);
    });

    sensor_pipeline_->SetLidarOdomCallback([this](const NavState& lo_state) {
        localizer_->FeedLidarOdom(lo_state);
        fusion_engine_->FeedLidarOdom(lo_state);
    });

    sensor_pipeline_->SetKeyframeScanCallback([this](CloudPtr scan) {
        if (options_.online_mode_) {
            lidar_loc_proc_cloud_.AddMessage(scan);
        } else {
            LidarLocProcCloud(scan);
        }
    });

    ///  各模块的异步调用
    options_.enable_lidar_loc_skip_ = yaml.GetValue<bool>("system", "enable_lidar_loc_skip");
    options_.enable_lidar_loc_rviz_ = yaml.GetValue<bool>("system", "enable_lidar_loc_rviz");
    options_.lidar_loc_skip_num_ = yaml.GetValue<int>("system", "lidar_loc_skip_num");
    lidar_loc_proc_cloud_.SetMaxSize(1);

    lidar_loc_proc_cloud_.SetName("激光定位");

    // 允许跳帧
    lidar_loc_proc_cloud_.SetSkipParam(options_.enable_lidar_loc_skip_, options_.lidar_loc_skip_num_);
    lidar_loc_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarLocProcCloud(cloud); });

    if (options_.online_mode_) {
        lidar_loc_proc_cloud_.Start();
        sensor_pipeline_->Start();
    }

    /// TODO: 发布
    fusion_engine_->SetHighFrequencyOutputCallback([this](const LocalizationResult& res) {
        if (loc_result_.timestamp_ > 0) {
            double loc_fps = 1.0 / (res.timestamp_ - loc_result_.timestamp_);
            LOG_EVERY_N(INFO, 10) << "loc fps: " << loc_fps;
        }

        loc_result_ = res;

        if (tf_callback_ && loc_result_.valid_) {
            tf_callback_(loc_result_.ToGeoMsg());
        }

        if (ui_) {
            ui_->UpdateNavState(loc_result_.ToNavState());
            ui_->UpdateRecentPose(loc_result_.pose_);
        }
    });
    return true;
}

void Localization::ProcessLidarMsg(const sensor_msgs::PointCloud2::ConstPtr cloud) {
    UL lock(global_mutex_);
    if (localizer_ == nullptr || sensor_pipeline_ == nullptr || fusion_engine_ == nullptr) {
        return;
    }

    sensor_pipeline_->ProcessPointCloud2(cloud);
}

void Localization::ProcessLivoxLidarMsg(const livox_ros_driver::CustomMsg::ConstPtr cloud) {
    UL lock(global_mutex_);
    if (localizer_ == nullptr || sensor_pipeline_ == nullptr || fusion_engine_ == nullptr) {
        return;
    }

    sensor_pipeline_->ProcessLivoxCloud(cloud);
}

void Localization::LidarLocProcCloud(CloudPtr scan_undist) {
    localizer_->ProcessKeyframeScan(scan_undist);

    auto res = localizer_->GetLocalizationResult();
    fusion_engine_->FeedLocalization(res);

    if (ui_) {
        // Twi with Til, here pose means Twl, thus Til=I
        ui_->UpdateScan(scan_undist, res.pose_);
    }

    if (loc_state_callback_) {
        std_msgs::Int32 loc_state;
        loc_state.data = static_cast<int>(res.status_);
        LOG(INFO) << "loc_state: " << loc_state.data;
        loc_state_callback_(loc_state);
    }
}

void Localization::ProcessIMUMsg(IMUPtr imu) {
    UL lock(global_mutex_);

    if (localizer_ == nullptr || sensor_pipeline_ == nullptr || fusion_engine_ == nullptr) {
        return;
    }

    double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    sensor_pipeline_->ProcessIMU(imu);
}

// void Localization::ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
//     UL lock(global_mutex_);
//
//     if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
//         return;
//     }
//     double this_odom_time = ToSec(odom_msg->header.stamp);
//     if (last_odom_time_ > 0 && this_odom_time < last_odom_time_) {
//         LOG(WARNING) << "Odom Time Abnormal:" << this_odom_time << ", last: " << last_odom_time_;
//     }
//     last_odom_time_ = this_odom_time;
//
//     lio_->ProcessOdometry(odom_msg);
//
//     if (!lio_->GetbOdomHF()) {
//         return;
//     }
//
//     auto dr_state = lio_->GetStateHF(mapping::FasterLioMapping::kHFStateOdomFiltered);
//
//     constexpr auto kThVbrbStill = 0.03;  // 0.08;
//     constexpr auto kThOmegaStill = 0.03;
//     if (dr_state.Getvwi().norm() < kThVbrbStill && dr_state.Getwii().norm() < kThOmegaStill) {
//         dr_state.is_parking_ = true;
//         dr_state.Setvwi(Vec3d::Zero());
//         dr_state.Setwii(Vec3d::Zero());
//     }
//
//     lidar_loc_->ProcessDR(dr_state);
//     pgo_->ProcessDR(dr_state);
// }

void Localization::Finish() {
    if (sensor_pipeline_) {
        sensor_pipeline_->Finish();
    }
    if (localizer_) {
        localizer_->Finish();
    }
    if (ui_) {
        ui_->Quit();
    }

    lidar_loc_proc_cloud_.Quit();
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    /// 设置外部重定位的pose
    if (localizer_) {
        localizer_->SetInitialPose(SE3(q, t));
    }
}

void Localization::SetTFCallback(Localization::TFCallback&& callback) { tf_callback_ = callback; }

}  // namespace lightning::loc
