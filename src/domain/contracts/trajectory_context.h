#pragma once

#include <memory>
#include <string>

#include "domain/contracts/event_sink.h"
#include "domain/geometry/pose3.h"
#include "domain/result/localization_result.h"
#include "domain/result/state_estimate.h"
#include "domain/sensor/cloud_data.h"
#include "domain/sensor/imu_data.h"

namespace lightning::domain::contracts {

class ITrajectoryContext {
   public:
    virtual ~ITrajectoryContext() = default;

    virtual std::string GetId() const = 0;
    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual void FeedImu(const sensor::ImuData& imu) = 0;
    virtual void FeedCloud(const sensor::CloudData& cloud) = 0;
    virtual void SetInitialPose(const geometry::Pose3& pose) = 0;
    virtual void SetEventSink(std::shared_ptr<IEventSink> sink) = 0;
    virtual result::StateEstimate GetLatestStateEstimate() const = 0;
    virtual result::LocalizationResult GetLatestLocalizationResult() const = 0;
};

}  // namespace lightning::domain::contracts
