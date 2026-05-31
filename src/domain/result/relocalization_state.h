#pragma once

namespace lightning::domain::result {

enum class LocalizationMode {
    kUninitialized,
    kGlobalInitialization,
    kLocalTracking,
};

enum class RelocalizationState {
    kIdle,
    kAccumulating,
    kGlobalInitializing,
    kTracking,
    kLost,
    kFailed,
    kRecovering,
};

}  // namespace lightning::domain::result
