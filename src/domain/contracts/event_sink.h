#pragma once

#include "domain/geometry/pose3.h"
#include "domain/result/localization_result.h"
#include "domain/result/map_state.h"
#include "domain/result/motion_estimate.h"
#include "domain/result/relocalization_state.h"
#include "domain/result/state_estimate.h"
#include "domain/sensor/cloud_data.h"

namespace lightning::domain::contracts {

class IEventSink {
   public:
    virtual ~IEventSink() = default;

    virtual void OnMotionEstimate(const result::MotionEstimate& estimate) = 0;
    virtual void OnStateEstimate(const result::StateEstimate& estimate) = 0;
    virtual void OnLocalizationResult(const result::LocalizationResult& result) = 0;
    virtual void OnRelocalizationState(result::RelocalizationState state) = 0;
    virtual void OnMapState(const result::MapState& map_state) = 0;
    virtual void OnCloudInWorld(const sensor::CloudData& cloud, const geometry::Pose3& pose) = 0;
};

}  // namespace lightning::domain::contracts
