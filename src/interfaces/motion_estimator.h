#pragma once

// Legacy compatibility interface for current localization builder / adapters.
// New stable contracts should exchange domain/result/motion_estimate.h values
// rather than depending on this algorithm-facing boundary.

#include <string>

#include "common/imu.h"
#include "common/keyframe.h"
#include "domain/result/motion_estimate.h"

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
    virtual CloudPtr GetProjectedCloud() const = 0;
};

}  // namespace lightning::loc
