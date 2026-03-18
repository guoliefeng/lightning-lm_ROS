#pragma once

#include <functional>

#include "common/nav_state.h"
#include "core/localization/localization_result.h"

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
