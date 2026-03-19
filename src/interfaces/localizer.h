#pragma once

#include <memory>
#include <string>

#include "common/nav_state.h"
#include "common/point_def.h"
#include "core/localization/localization_result.h"

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::loc {

class ILocalizer {
   public:
    virtual ~ILocalizer() = default;

    virtual bool Init(const std::string& yaml_path) = 0;
    virtual void FeedLidarOdom(const NavState& state) = 0;
    virtual void FeedDeadReckoning(const NavState& state) = 0;
    virtual bool ProcessKeyframeScan(CloudPtr cloud) = 0;
    virtual void SetUI(std::shared_ptr<ui::PangolinWindow> ui) = 0;
    virtual void SetInitialPose(const SE3& pose) = 0;
    virtual LocalizationResult GetLocalizationResult() const = 0;
    virtual void Finish() = 0;
};

}  // namespace lightning::loc
