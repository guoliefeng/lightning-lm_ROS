#include "bridges/localization_bridge.h"

#include "core/lio/pointcloud_preprocess.h"

namespace lightning::loc {

namespace {

SensorCloudInput MakeSensorCloudInput(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    SensorCloudInput input;
    if (cloud == nullptr) {
        return input;
    }

    input.stamp_ns = static_cast<std::uint64_t>(cloud->header.stamp.sec) * 1000000000ULL + cloud->header.stamp.nsec;
    input.converter = [cloud](PointCloudPreprocess& preprocess, CloudPtr& laser_cloud) {
        preprocess.Process(cloud, laser_cloud);
    };
    return input;
}

SensorCloudInput MakeSensorCloudInput(const livox_ros_driver::CustomMsg::ConstPtr& cloud) {
    SensorCloudInput input;
    if (cloud == nullptr) {
        return input;
    }

    input.stamp_ns = static_cast<std::uint64_t>(cloud->header.stamp.sec) * 1000000000ULL + cloud->header.stamp.nsec;
    input.converter = [cloud](PointCloudPreprocess& preprocess, CloudPtr& laser_cloud) {
        preprocess.Process(cloud, laser_cloud);
    };
    return input;
}

}  // namespace

LocalizationBridge::LocalizationBridge(std::shared_ptr<ILocalizationRuntime> runtime) : runtime_(std::move(runtime)) {}

bool LocalizationBridge::Init(const std::string& yaml_path, const std::string& global_map_path) {
    if (runtime_ == nullptr) {
        return false;
    }
    return runtime_->Init(yaml_path, global_map_path);
}

void LocalizationBridge::ProcessIMU(const IMUPtr& imu) {
    if (runtime_) {
        runtime_->FeedImu(imu);
    }
}

void LocalizationBridge::ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (runtime_) {
        runtime_->FeedCloud(MakeSensorCloudInput(cloud));
    }
}

void LocalizationBridge::ProcessLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud) {
    if (runtime_) {
        runtime_->FeedCloud(MakeSensorCloudInput(cloud));
    }
}

void LocalizationBridge::SetInitialPose(const SE3& pose) {
    if (runtime_) {
        runtime_->SetInitialPose(pose);
    }
}

void LocalizationBridge::Finish() {
    if (runtime_) {
        runtime_->Finish();
    }
}

void LocalizationBridge::SetTFCallback(ILocalizationRuntime::TFCallback cb) {
    if (runtime_) {
        runtime_->SetTFCallback(std::move(cb));
    }
}

void LocalizationBridge::SetLocStateCallback(ILocalizationRuntime::LocStateCallback cb) {
    if (runtime_) {
        runtime_->SetLocStateCallback(std::move(cb));
    }
}

}  // namespace lightning::loc
