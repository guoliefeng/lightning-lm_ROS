#pragma once

#include <functional>

#include <sensor_msgs/PointCloud2.h>

#include "common/imu.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "livox_ros_driver/CustomMsg.h"

namespace lightning::loc {

class ISensorPipeline {
   public:
    using DeadReckoningCallback = std::function<void(const NavState&)>;
    using LidarOdomCallback = std::function<void(const NavState&)>;
    using KeyframeScanCallback = std::function<void(CloudPtr)>;

    virtual ~ISensorPipeline() = default;

    virtual void SetDeadReckoningCallback(DeadReckoningCallback cb) = 0;
    virtual void SetLidarOdomCallback(LidarOdomCallback cb) = 0;
    virtual void SetKeyframeScanCallback(KeyframeScanCallback cb) = 0;

    virtual void Start() = 0;
    virtual void Finish() = 0;

    virtual void ProcessIMU(IMUPtr imu) = 0;
    virtual void ProcessCloud(CloudPtr cloud) = 0;
    virtual void ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud) = 0;
    virtual void ProcessLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud) = 0;
};

}  // namespace lightning::loc
