#include "bridges/localization_bridge_ros1.h"

#include "bridges/sensor_conversion.h"

namespace lightning::loc {

LocalizationBridgeRos1::LocalizationBridgeRos1(std::shared_ptr<ILocalizationRuntime> runtime)
    : runtime_(std::move(runtime)) {}

bool LocalizationBridgeRos1::Init(const std::string& yaml_path, const std::string& global_map_path) {
    if (runtime_ == nullptr) {
        return false;
    }
    return runtime_->Init(yaml_path, global_map_path);
}

void LocalizationBridgeRos1::ProcessIMU(const sensor_msgs::Imu::ConstPtr& imu) {
    if (runtime_) {
        runtime_->FeedImu(bridges::ConvertRosImuToInternalImu(imu));
    }
}

void LocalizationBridgeRos1::ProcessIMU(const IMUPtr& imu) {
    if (runtime_) {
        runtime_->FeedImu(imu);
    }
}

void LocalizationBridgeRos1::ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (runtime_) {
        runtime_->FeedCloud(bridges::ConvertPointCloud2ToSensorCloudInput(cloud));
    }
}

void LocalizationBridgeRos1::ProcessLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud) {
    if (runtime_) {
        runtime_->FeedCloud(bridges::ConvertLivoxToSensorCloudInput(cloud));
    }
}

void LocalizationBridgeRos1::SetInitialPose(const SE3& pose) {
    if (runtime_) {
        runtime_->SetInitialPose(pose);
    }
}

void LocalizationBridgeRos1::Finish() {
    if (runtime_) {
        runtime_->Finish();
    }
}

void LocalizationBridgeRos1::SetTFCallback(ILocalizationRuntime::TFCallback cb) {
    if (runtime_) {
        runtime_->SetTFCallback(std::move(cb));
    }
}

void LocalizationBridgeRos1::SetLocStateCallback(ILocalizationRuntime::LocStateCallback cb) {
    if (runtime_) {
        runtime_->SetLocStateCallback(std::move(cb));
    }
}

}  // namespace lightning::loc
