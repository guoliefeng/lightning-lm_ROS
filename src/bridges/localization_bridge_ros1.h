#pragma once

#include <memory>
#include <string>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "common/imu.h"
#include "interfaces/localization_runtime.h"
#include "livox_ros_driver/CustomMsg.h"

namespace lightning::loc {

class LocalizationBridgeRos1 {
   public:
    explicit LocalizationBridgeRos1(std::shared_ptr<ILocalizationRuntime> runtime);

    bool Init(const std::string& yaml_path, const std::string& global_map_path);
    void ProcessIMU(const sensor_msgs::Imu::ConstPtr& imu);
    void ProcessIMU(const IMUPtr& imu);
    void ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void ProcessLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud);
    void SetInitialPose(const SE3& pose);
    void Finish();

    void SetTFCallback(ILocalizationRuntime::TFCallback cb);
    void SetLocStateCallback(ILocalizationRuntime::LocStateCallback cb);

   private:
    std::shared_ptr<ILocalizationRuntime> runtime_ = nullptr;
};

}  // namespace lightning::loc
