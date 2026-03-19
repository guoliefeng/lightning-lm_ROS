#pragma once

#include <memory>
#include <string>

#include "interfaces/fusion_engine.h"
#include "interfaces/localizer.h"
#include "interfaces/motion_estimator.h"

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::loc {

struct LocalizationComponents {
    std::shared_ptr<IMotionEstimator> motion_estimator = nullptr;
    std::shared_ptr<ILocalizer> localizer = nullptr;
    std::shared_ptr<IFusionEngine> fusion_engine = nullptr;
};

class LocalizationBuilder {
   public:
    static LocalizationComponents BuildLocalizationComponents(const std::string& yaml_path,
                                                             const std::string& global_map_path,
                                                             bool with_ui,
                                                             std::shared_ptr<ui::PangolinWindow> ui);
};

}  // namespace lightning::loc
