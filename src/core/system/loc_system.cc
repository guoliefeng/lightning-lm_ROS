//
// Created by xiang on 25-9-12.
//
#include <cmath>
#include <csignal>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "bridges/localization_bridge_ros1.h"
#include "common/options.h"
#include "core/lightning_math.hpp"
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
    loc_bridge_ = std::make_shared<loc::LocalizationBridgeRos1>(runtime);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros1 node ... ";

    /// subscribers
    node_ = std::make_shared<ros::NodeHandle>();

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    const YAML::Node yaml_node = YAML::LoadFile(yaml_path);
    if (yaml_node["common"] && yaml_node["common"]["init_pose_topic"]) {
        init_pose_topic_ = yaml_node["common"]["init_pose_topic"].as<std::string>();
        use_init_pose_topic_ = !init_pose_topic_.empty();
    }
    if (yaml_node["common"] && yaml_node["common"]["imu_to_base_rotation"]) {
        const auto rotation_values =
            yaml_node["common"]["imu_to_base_rotation"].as<std::vector<double>>();
        if (rotation_values.size() != 9) {
            LOG(ERROR) << "common.imu_to_base_rotation must contain 9 values";
            return false;
        }

        imu_to_base_rotation_ = math::MatFromArray<double>(rotation_values);
        const Mat3d should_be_identity = imu_to_base_rotation_ * imu_to_base_rotation_.transpose();
        if (!should_be_identity.isApprox(Mat3d::Identity(), 1e-6) ||
            std::abs(imu_to_base_rotation_.determinant() - 1.0) > 1e-6) {
            LOG(ERROR) << "common.imu_to_base_rotation is not a valid rotation matrix";
            return false;
        }
        LOG(INFO) << "IMU to base_link rotation:\n" << imu_to_base_rotation_;
    }

    imu_sub_ = node_->subscribe<sensor_msgs::Imu>(
        imu_topic_, 10, [this](const sensor_msgs::Imu::ConstPtr& msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            const Vec3d acceleration_imu(
                msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            const Vec3d angular_velocity_imu(
                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            imu->linear_acceleration = imu_to_base_rotation_ * acceleration_imu;
            imu->angular_velocity = imu_to_base_rotation_ * angular_velocity_imu;
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

    if (use_init_pose_topic_) {
        init_pose_sub_ = node_->subscribe<nav_msgs::Odometry>(
            init_pose_topic_, 10, [this](const nav_msgs::Odometry::ConstPtr& odom) { ProcessInitPose(odom); });
        LOG(INFO) << "waiting init pose from " << init_pose_topic_;
    }

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

void LocSystem::ProcessInitPose(const nav_msgs::Odometry::ConstPtr &odom) {
    if (loc_started_) {
        return;
    }

    const auto &p = odom->pose.pose.position;
    const auto &q = odom->pose.pose.orientation;
    Quatd quat(q.w, q.x, q.y, q.z);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(quat.w()) || !std::isfinite(quat.x()) ||
        !std::isfinite(quat.y()) || !std::isfinite(quat.z()) || quat.norm() < 1e-6) {
        LOG(ERROR) << "invalid init pose from " << init_pose_topic_;
        return;
    }

    quat.normalize();
    SetInitPose(SE3(quat, Vec3d(p.x, p.y, p.z)));
    init_pose_sub_.shutdown();
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
