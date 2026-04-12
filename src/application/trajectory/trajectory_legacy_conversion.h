#pragma once

#include <memory>
#include <string>

#include "common/imu.h"
#include "common/nav_state.h"
#include "common/sensor_cloud_input.h"
#include "core/localization/localization_result.h"
#include "domain/geometry/pose3.h"
#include "domain/result/localization_result.h"
#include "domain/result/motion_estimate.h"
#include "domain/sensor/cloud_data.h"
#include "domain/sensor/imu_data.h"

namespace lightning {
class PointCloudPreprocess;
}

namespace lightning::application::trajectory::legacy {

class LegacyCloudConverter {
   public:
    explicit LegacyCloudConverter(const std::string& yaml_path);

    bool IsValid() const;
    domain::sensor::CloudData ToDomainCloud(const SensorCloudInput& cloud_input) const;

   private:
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;
};

domain::geometry::Pose3 ToPose3(const SE3& pose);
domain::result::MotionEstimate ToMotionEstimate(const NavState& state, domain::result::MotionEstimateSource source);
domain::result::LocalizationResult ToLocalizationResult(const loc::LocalizationResult& result);
IMUPtr ToLegacyImu(const domain::sensor::ImuData& imu);
domain::sensor::ImuData ToDomainImu(const IMUPtr& imu);
SensorCloudInput ToLegacyCloud(const domain::sensor::CloudData& cloud);

}  // namespace lightning::application::trajectory::legacy
