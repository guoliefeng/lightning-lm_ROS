#include "bridges/sensor_conversion.h"

#include "core/lio/pointcloud_preprocess.h"
#include "wrapper/ros_utils.h"

namespace lightning::loc::bridges {

SensorCloudInput ConvertPointCloud2ToSensorCloudInput(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
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

SensorCloudInput ConvertLivoxToSensorCloudInput(const livox_ros_driver::CustomMsg::ConstPtr& cloud) {
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

IMUPtr ConvertRosImuToInternalImu(const sensor_msgs::Imu::ConstPtr& msg) {
    if (msg == nullptr) {
        return nullptr;
    }

    IMUPtr imu = std::make_shared<IMU>();
    imu->timestamp = ToSec(msg->header.stamp);
    imu->linear_acceleration =
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    return imu;
}

}  // namespace lightning::loc::bridges
