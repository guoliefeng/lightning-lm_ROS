#include "core/localization/localization_runtime_factory.h"

#include "core/localization/localization.h"

namespace lightning::loc {

std::shared_ptr<ILocalizationRuntime> CreateLocalizationRuntime(const LocalizationRuntimeOptions& options) {
    Localization::Options localization_options;
    localization_options.online_mode_ = options.online_mode_;
    localization_options.with_ui_ = options.with_ui_;
    return std::make_shared<Localization>(localization_options);
}

}  // namespace lightning::loc
