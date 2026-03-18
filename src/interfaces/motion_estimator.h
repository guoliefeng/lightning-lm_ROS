#pragma once

#include <string>

#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning::loc {

class IMotionEstimator {
   public:
    virtual ~IMotionEstimator() = default;

    virtual bool Init(const std::string& yaml_path) = 0;
    virtual void ProcessIMU(const IMUPtr& imu) = 0;
    virtual void ProcessCloud(CloudPtr cloud) = 0;
    virtual bool Run() = 0;
    virtual NavState GetLidarOdomState() const = 0;
    virtual NavState GetDeadReckoningState() const = 0;
    virtual Keyframe::Ptr GetKeyframe() const = 0;
    virtual CloudPtr GetUndistortedScan() const = 0;
};

}  // namespace lightning::loc
