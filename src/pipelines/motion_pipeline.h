#pragma once

#include <memory>

#include "core/lio/pointcloud_preprocess.h"
#include "core/system/async_message_process.h"
#include "interfaces/motion_estimator.h"
#include "interfaces/sensor_pipeline.h"

namespace lightning::loc {

class MotionPipeline : public ISensorPipeline {
   public:
    struct Options {
        bool online_mode_ = false;
        bool enable_lidar_odom_skip_ = false;
        int lidar_odom_skip_num_ = 1;
    };

    MotionPipeline(Options options, std::shared_ptr<IMotionEstimator> motion_estimator,
                   std::shared_ptr<PointCloudPreprocess> preprocess);

    void SetDeadReckoningCallback(DeadReckoningCallback cb) override;
    void SetLidarOdomCallback(LidarOdomCallback cb) override;
    void SetKeyframeScanCallback(KeyframeScanCallback cb) override;

    void Start() override;
    void Finish() override;

    void ProcessIMU(IMUPtr imu) override;
    void ProcessCloud(CloudPtr cloud) override;
    void ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud) override;
    void ProcessLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud) override;

   private:
    void HandleCloud(CloudPtr cloud);

    Options options_;
    std::shared_ptr<IMotionEstimator> motion_estimator_ = nullptr;
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;
    Keyframe::Ptr last_keyframe_ = nullptr;

    DeadReckoningCallback dead_reckoning_callback_;
    LidarOdomCallback lidar_odom_callback_;
    KeyframeScanCallback keyframe_scan_callback_;

    sys::AsyncMessageProcess<CloudPtr> lidar_odom_proc_cloud_;
};

}  // namespace lightning::loc
