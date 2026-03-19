#include "pipelines/motion_pipeline.h"

namespace lightning::loc {

MotionPipeline::MotionPipeline(Options options, std::shared_ptr<IMotionEstimator> motion_estimator,
                               std::shared_ptr<PointCloudPreprocess> preprocess)
    : options_(options), motion_estimator_(std::move(motion_estimator)), preprocess_(std::move(preprocess)) {
    lidar_odom_proc_cloud_.SetMaxSize(1);
    lidar_odom_proc_cloud_.SetName("激光里程计");
    lidar_odom_proc_cloud_.SetSkipParam(options_.enable_lidar_odom_skip_, options_.lidar_odom_skip_num_);
    lidar_odom_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { HandleCloud(cloud); });
}

void MotionPipeline::SetDeadReckoningCallback(DeadReckoningCallback cb) { dead_reckoning_callback_ = std::move(cb); }

void MotionPipeline::SetLidarOdomCallback(LidarOdomCallback cb) { lidar_odom_callback_ = std::move(cb); }

void MotionPipeline::SetKeyframeScanCallback(KeyframeScanCallback cb) { keyframe_scan_callback_ = std::move(cb); }

void MotionPipeline::Start() {
    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.Start();
    }
}

void MotionPipeline::Finish() { lidar_odom_proc_cloud_.Quit(); }

void MotionPipeline::ProcessIMU(IMUPtr imu) {
    if (motion_estimator_ == nullptr) {
        return;
    }

    motion_estimator_->ProcessIMU(imu);

    auto dr_state = motion_estimator_->GetDeadReckoningState();
    if (!dr_state.pose_is_ok_) {
        return;
    }

    if (dead_reckoning_callback_) {
        dead_reckoning_callback_(dr_state);
    }
}

void MotionPipeline::ProcessCloud(CloudPtr cloud) {
    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.AddMessage(cloud);
    } else {
        HandleCloud(cloud);
    }
}

void MotionPipeline::ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (preprocess_ == nullptr) {
        return;
    }

    CloudPtr laser_cloud(new PointCloudType);
    preprocess_->Process(cloud, laser_cloud);
    laser_cloud->header.stamp = cloud->header.stamp.sec * 1e9 + cloud->header.stamp.nsec;
    ProcessCloud(laser_cloud);
}

void MotionPipeline::ProcessLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud) {
    if (preprocess_ == nullptr) {
        return;
    }

    CloudPtr laser_cloud(new PointCloudType);
    preprocess_->Process(cloud, laser_cloud);
    laser_cloud->header.stamp = cloud->header.stamp.sec * 1e9 + cloud->header.stamp.nsec;
    ProcessCloud(laser_cloud);
}

void MotionPipeline::HandleCloud(CloudPtr cloud) {
    if (motion_estimator_ == nullptr) {
        return;
    }

    motion_estimator_->ProcessCloud(cloud);
    if (!motion_estimator_->Run()) {
        return;
    }

    auto lo_state = motion_estimator_->GetLidarOdomState();
    if (lidar_odom_callback_) {
        lidar_odom_callback_(lo_state);
    }

    auto kf = motion_estimator_->GetKeyframe();
    if (kf == last_keyframe_) {
        return;
    }

    last_keyframe_ = kf;

    if (keyframe_scan_callback_) {
        keyframe_scan_callback_(motion_estimator_->GetUndistortedScan());
    }
}

}  // namespace lightning::loc
