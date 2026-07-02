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

domain::sensor::GnssData ConvertNavSatFixToGnssData(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    domain::sensor::GnssData gnss;
    if (msg == nullptr) {
        return gnss;
    }

    gnss.stamp_ns = static_cast<std::uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nsec;
    gnss.sensor_id = msg->header.frame_id;
    gnss.latitude_deg = msg->latitude;
    gnss.longitude_deg = msg->longitude;
    gnss.altitude_m = msg->altitude;

    switch (msg->status.status) {
        case sensor_msgs::NavSatStatus::STATUS_FIX:
            gnss.status = domain::sensor::GnssFixStatus::kFix;
            break;
        case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
            gnss.status = domain::sensor::GnssFixStatus::kSbasFix;
            break;
        case sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
            gnss.status = domain::sensor::GnssFixStatus::kGbasFix;
            break;
        default:
            gnss.status = domain::sensor::GnssFixStatus::kNoFix;
            break;
    }

    if (msg->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                gnss.position_covariance(r, c) = msg->position_covariance[r * 3 + c];
            }
        }
        gnss.covariance_valid = true;
    }
    return gnss;
}

domain::sensor::OdometryData ConvertRosOdometryToOdometryData(const nav_msgs::Odometry::ConstPtr& msg) {
    domain::sensor::OdometryData odom;
    if (msg == nullptr) {
        return odom;
    }

    odom.stamp_ns = static_cast<std::uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nsec;
    odom.sensor_id = msg->child_frame_id;
    odom.frame_id = msg->header.frame_id;
    odom.child_frame_id = msg->child_frame_id;

    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    odom.pose.translation = Eigen::Vector3d(p.x, p.y, p.z);
    odom.pose.rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    odom.pose_valid = std::abs(odom.pose.rotation.norm() - 1.0) < 0.1;
    if (odom.pose_valid) {
        odom.pose.rotation.normalize();
    }

    const auto& lv = msg->twist.twist.linear;
    const auto& av = msg->twist.twist.angular;
    odom.linear_velocity = Eigen::Vector3d(lv.x, lv.y, lv.z);
    odom.angular_velocity = Eigen::Vector3d(av.x, av.y, av.z);
    odom.twist_valid = true;
    return odom;
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
