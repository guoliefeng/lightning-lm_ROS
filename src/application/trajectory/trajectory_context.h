#pragma once

#include <memory>
#include <string>

#include "domain/contracts/event_sink.h"
#include "domain/result/localization_result.h"
#include "domain/result/state_estimate.h"
#include "domain/sensor/cloud_data.h"
#include "domain/sensor/imu_data.h"

namespace lightning::application::trajectory {

class ITrajectoryContext {
   public:
    virtual ~ITrajectoryContext() = default;

    virtual std::string GetId() const = 0;
    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual void FeedImu(const domain::sensor::ImuData& imu) = 0;
    virtual void FeedCloud(const domain::sensor::CloudData& cloud) = 0;
    virtual void SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) = 0;
    virtual domain::result::StateEstimate GetLatestStateEstimate() const = 0;
    virtual domain::result::LocalizationResult GetLatestLocalizationResult() const = 0;
};

}  // namespace lightning::application::trajectory
