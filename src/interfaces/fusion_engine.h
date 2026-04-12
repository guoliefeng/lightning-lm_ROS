#pragma once

// Legacy compatibility interface.
// New stable contracts should depend on domain/contracts/state_estimator.h
// and domain/contracts/pose_graph_backend.h instead of this composite facade.

#include <functional>

#include "common/nav_state.h"
#include "core/localization/localization_result.h"
#include "domain/contracts/pose_graph_backend.h"
#include "domain/contracts/state_estimator.h"

namespace lightning::loc {

class IFusionEngine {
   public:
    using OutputCallback = std::function<void(const LocalizationResult& result)>;

    virtual ~IFusionEngine() = default;

    virtual void FeedDeadReckoning(const NavState& state) = 0;
    virtual void FeedLidarOdom(const NavState& state) = 0;
    virtual void FeedLocalization(const LocalizationResult& result) = 0;
    virtual void SetHighFrequencyOutputCallback(OutputCallback cb) = 0;
};

}  // namespace lightning::loc
