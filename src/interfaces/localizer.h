#pragma once

// Legacy compatibility interface.
// UI updates are emitted through runtime events instead of this algorithm interface.

#include <memory>
#include <string>

#include "common/nav_state.h"
#include "common/point_def.h"
#include "core/localization/localization_result.h"
#include "domain/contracts/event_sink.h"

namespace lightning::loc {

class ILocalizer {
   public:
    virtual ~ILocalizer() = default;

    virtual bool Init(const std::string& yaml_path) = 0;
    virtual void FeedLidarOdom(const NavState& state) = 0;
    virtual void FeedDeadReckoning(const NavState& state) = 0;
    virtual bool ProcessKeyframeScan(CloudPtr cloud) = 0;
    virtual void SetInitialPose(const SE3& pose) = 0;
    virtual LocalizationResult GetLocalizationResult() const = 0;
    virtual void Finish() = 0;
};

}  // namespace lightning::loc
