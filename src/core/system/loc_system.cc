//
// Created by xiang on 25-9-12.
//
#include <csignal>

#include "bridges/localization_bridge.h"
#include "core/localization/localization_runtime_factory.h"
#include "core/system/loc_system.h"
#include "io/yaml_io.h"
#include "utils/timer.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() {
    if (loc_bridge_) {
        loc_bridge_->Finish();
    }
}

bool LocSystem::Init(const std::string &yaml_path) {
    loc::LocalizationRuntimeOptions opt;
    opt.online_mode_ = true;
    auto runtime = loc::CreateLocalizationRuntime(opt);
    loc_bridge_ = std::make_shared<loc::LocalizationBridge>(runtime);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros1 node ... ";

    /// subscribers
    node_ = std::make_shared<ros::NodeHandle>();

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    imu_sub_ = node_->subscribe<sensor_msgs::Imu>(
        imu_topic_, 10, [this](const sensor_msgs::Imu::ConstPtr& msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->subscribe<sensor_msgs::PointCloud2>(
        cloud_topic_, 10, [this](const sensor_msgs::PointCloud2::ConstPtr& cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->subscribe<livox_ros_driver::CustomMsg>(
        livox_topic_, 10, [this](const livox_ros_driver::CustomMsg::ConstPtr& cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        loc_bridge_->SetTFCallback(
            [this](const geometry_msgs::TransformStamped &pose) { tf_broadcaster_->sendTransform(pose); });
    }

    bool ret = loc_bridge_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_bridge_->SetInitialPose(pose);
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_bridge_->ProcessIMU(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    if (loc_started_) {
        loc_bridge_->ProcessPointCloud2(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver::CustomMsg::ConstPtr &cloud) {
    if (loc_started_) {
        loc_bridge_->ProcessLivoxCloud(cloud);
    }
}

void LocSystem::Spin() { if (node_ != nullptr) { ros::spin(); } }

}  // namespace lightning
