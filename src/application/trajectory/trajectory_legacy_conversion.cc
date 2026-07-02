#include "application/trajectory/trajectory_legacy_conversion.h"

#include <glog/logging.h>

#include "core/lio/pointcloud_preprocess.h"
#include "io/yaml_io.h"

namespace lightning::application::trajectory::legacy {

namespace {

std::shared_ptr<PointCloudPreprocess> CreateConfiguredPreprocess(const std::string& yaml_path) {
    YAML_IO yaml(yaml_path);
    if (!yaml.IsOpened()) {
        LOG(ERROR) << "failed to open yaml for legacy cloud converter: " << yaml_path;
        return nullptr;
    }

    auto preprocess = std::make_shared<PointCloudPreprocess>();
    preprocess->Blind() = yaml.GetValue<double>("fasterlio", "blind");
    preprocess->TimeScale() = yaml.GetValue<double>("fasterlio", "time_scale");
    preprocess->NumScans() = yaml.GetValue<int>("fasterlio", "scan_line");
    preprocess->PointFilterNum() = yaml.GetValue<int>("fasterlio", "point_filter_num");
    preprocess->SetHeightROI(yaml.GetValue<float>("roi", "height_max"), yaml.GetValue<float>("roi", "height_min"));

    int lidar_type = yaml.GetValue<int>("fasterlio", "lidar_type");
    if (lidar_type == 1) {
        preprocess->SetLidarType(LidarType::AVIA);
    } else if (lidar_type == 2) {
        preprocess->SetLidarType(LidarType::VELO32);
    } else if (lidar_type == 3) {
        preprocess->SetLidarType(LidarType::OUST64);
    } else if (lidar_type == 4) {
        preprocess->SetLidarType(LidarType::ROBOSENSE);
    } else if (lidar_type == 6) {
        preprocess->SetLidarType(LidarType::MERGED);
    }

    return preprocess;
}

domain::sensor::CloudData ToDomainCloudFromPcl(const CloudPtr& cloud, std::uint64_t stamp_ns) {
    domain::sensor::CloudData converted;
    if (cloud == nullptr) {
        return converted;
    }

    converted.stamp_ns = stamp_ns;
    converted.is_dense = cloud->is_dense;
    converted.points.reserve(cloud->size());
    for (const auto& point : cloud->points) {
        domain::sensor::CloudPoint converted_point;
        converted_point.x = point.x;
        converted_point.y = point.y;
        converted_point.z = point.z;
        converted_point.intensity = point.intensity;
        converted_point.relative_time_s = point.time;
        converted.points.push_back(converted_point);
    }
    return converted;
}

}  // namespace

LegacyCloudConverter::LegacyCloudConverter(const std::string& yaml_path) : preprocess_(CreateConfiguredPreprocess(yaml_path)) {}

bool LegacyCloudConverter::IsValid() const { return preprocess_ != nullptr; }

domain::sensor::CloudData LegacyCloudConverter::ToDomainCloud(const SensorCloudInput& cloud_input) const {
    if (cloud_input.Empty()) {
        return {};
    }

    if (cloud_input.cloud != nullptr) {
        return ToDomainCloudFromPcl(cloud_input.cloud, cloud_input.stamp_ns);
    }

    if (preprocess_ == nullptr || !cloud_input.converter) {
        return {};
    }

    CloudPtr cloud(new PointCloudType());
    cloud_input.converter(*preprocess_, cloud);
    return ToDomainCloudFromPcl(cloud, cloud_input.stamp_ns);
}

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

domain::result::LocalizationResult ToLocalizationResult(const loc::LocalizationResult& result) {
    domain::result::LocalizationResult converted;
    converted.timestamp_s = result.timestamp_;
    converted.pose = ToPose3(result.pose_);
    converted.valid = result.valid_;

    switch (result.status_) {
        case loc::LocalizationStatus::IDLE:
            converted.status = domain::result::LocalizationStatus::kIdle;
            break;
        case loc::LocalizationStatus::INITIALIZING:
            converted.status = domain::result::LocalizationStatus::kInitializing;
            break;
        case loc::LocalizationStatus::GOOD:
            converted.status = domain::result::LocalizationStatus::kGood;
            break;
        case loc::LocalizationStatus::FOLLOWING_DR:
            converted.status = domain::result::LocalizationStatus::kFollowingMotion;
            break;
        case loc::LocalizationStatus::FAIL:
            converted.status = domain::result::LocalizationStatus::kFail;
            break;
    }

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

domain::sensor::CloudData ToDomainCloud(const CloudPtr& cloud, std::uint64_t stamp_ns) {
    return ToDomainCloudFromPcl(cloud, stamp_ns);
}

SensorCloudInput ToLegacyCloud(const domain::sensor::CloudData& cloud) {
    SensorCloudInput converted;
    converted.stamp_ns = cloud.stamp_ns;
    converted.cloud.reset(new PointCloudType());
    // legacy 管线（LaserMapping/LidarLoc）通过 math::ToSec(header.stamp) 读取纳秒时间戳，
    // 不回填会导致 LIO 的 IMU/点云同步永远失败
    converted.cloud->header.stamp = cloud.stamp_ns;
    converted.cloud->reserve(cloud.points.size());
    for (const auto& point : cloud.points) {
        PointType converted_point;
        converted_point.x = point.x;
        converted_point.y = point.y;
        converted_point.z = point.z;
        converted_point.intensity = point.intensity;
        converted_point.time = point.relative_time_s;
        converted.cloud->push_back(converted_point);
    }
    converted.cloud->is_dense = cloud.is_dense;
    return converted;
}

}  // namespace lightning::application::trajectory::legacy
