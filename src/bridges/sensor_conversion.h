#pragma once

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "common/imu.h"
#include "common/sensor_cloud_input.h"
#include "domain/sensor/gnss_data.h"
#include "domain/sensor/odometry_data.h"
#include "livox_ros_driver/CustomMsg.h"

namespace lightning::loc::bridges {

SensorCloudInput ConvertPointCloud2ToSensorCloudInput(const sensor_msgs::PointCloud2::ConstPtr& cloud);
SensorCloudInput ConvertLivoxToSensorCloudInput(const livox_ros_driver::CustomMsg::ConstPtr& cloud);
IMUPtr ConvertRosImuToInternalImu(const sensor_msgs::Imu::ConstPtr& msg);
domain::sensor::GnssData ConvertNavSatFixToGnssData(const sensor_msgs::NavSatFix::ConstPtr& msg);
domain::sensor::OdometryData ConvertRosOdometryToOdometryData(const nav_msgs::Odometry::ConstPtr& msg);

}  // namespace lightning::loc::bridges
