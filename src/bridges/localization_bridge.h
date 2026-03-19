#pragma once

#include <memory>
#include <string>

#include "interfaces/localization_runtime.h"

namespace lightning::loc {

class LocalizationBridge {
   public:
    explicit LocalizationBridge(std::shared_ptr<ILocalizationRuntime> runtime);

    bool Init(const std::string& yaml_path, const std::string& global_map_path);
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
