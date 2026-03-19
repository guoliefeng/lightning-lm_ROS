#pragma once

#include <memory>

#include "interfaces/localization_runtime.h"

namespace lightning::loc {

struct LocalizationRuntimeOptions {
    bool online_mode_ = false;
    bool with_ui_ = false;
};

std::shared_ptr<ILocalizationRuntime> CreateLocalizationRuntime(const LocalizationRuntimeOptions& options);

}  // namespace lightning::loc
