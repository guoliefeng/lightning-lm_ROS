#pragma once

#include <functional>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "livox_ros_driver/CustomMsg.h"

namespace lightning::loc {

class ILocalizationRuntime {
   public:
    using TFCallback = std::function<void(const geometry_msgs::TransformStamped&)>;
    using LocStateCallback = std::function<void(const std_msgs::Int32&)>;

    virtual ~ILocalizationRuntime() = default;

    virtual bool Init(const std::string& yaml_path, const std::string& global_map_path) = 0;
    virtual void FeedImu(IMUPtr imu) = 0;
    virtual void FeedPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud) = 0;
    virtual void FeedLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud) = 0;
    virtual void SetInitialPose(const SE3& pose) = 0;
    virtual void Finish() = 0;

    virtual void SetTFCallback(TFCallback cb) = 0;
    virtual void SetLocStateCallback(LocStateCallback cb) = 0;
};

}  // namespace lightning::loc
