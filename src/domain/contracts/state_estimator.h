#pragma once

#include <functional>

#include "domain/result/localization_result.h"
#include "domain/result/motion_estimate.h"
#include "domain/result/state_estimate.h"
#include "domain/sensor/gnss_data.h"
#include "domain/sensor/odometry_data.h"

namespace lightning::domain::contracts {

class IStateEstimator {
   public:
    using OutputCallback = std::function<void(const result::StateEstimate&)>;

    virtual ~IStateEstimator() = default;

    virtual void FeedMotionEstimate(const result::MotionEstimate& motion) = 0;
    virtual void FeedLocalizationResult(const result::LocalizationResult& localization) = 0;
    // GNSS/轮速可选观测，未实现融合的估计器可忽略
    virtual void FeedGnss(const sensor::GnssData&) {}
    virtual void FeedOdometry(const sensor::OdometryData&) {}
    virtual void SetOutputCallback(OutputCallback callback) = 0;
    virtual result::StateEstimate GetLatestEstimate() const = 0;
    virtual void Reset() = 0;
};

}  // namespace lightning::domain::contracts
