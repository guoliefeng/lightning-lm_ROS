#pragma once

#include <functional>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/sensor_cloud_input.h"

namespace lightning::loc {

class ILocalizationRuntime {
   public:
    using TFCallback = std::function<void(const geometry_msgs::TransformStamped&)>;
    using LocStateCallback = std::function<void(const std_msgs::Int32&)>;

    virtual ~ILocalizationRuntime() = default;

    virtual bool Init(const std::string& yaml_path, const std::string& global_map_path) = 0;
    virtual void FeedImu(IMUPtr imu) = 0;
    virtual void FeedCloud(const SensorCloudInput& cloud) = 0;
    virtual void SetInitialPose(const SE3& pose) = 0;
    virtual void Finish() = 0;

    virtual void SetTFCallback(TFCallback cb) = 0;
    virtual void SetLocStateCallback(LocStateCallback cb) = 0;
};

}  // namespace lightning::loc
