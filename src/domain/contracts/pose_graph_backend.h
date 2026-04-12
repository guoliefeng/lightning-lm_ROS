#pragma once

#include <functional>

#include "domain/result/localization_result.h"
#include "domain/result/motion_estimate.h"

namespace lightning::domain::contracts {

class IPoseGraphBackend {
   public:
    using OutputCallback = std::function<void(const result::LocalizationResult&)>;

    virtual ~IPoseGraphBackend() = default;

    virtual void FeedMotionEstimate(const result::MotionEstimate& motion) = 0;
    virtual void FeedLocalizationResult(const result::LocalizationResult& localization) = 0;
    virtual void SetOutputCallback(OutputCallback callback) = 0;
    virtual result::LocalizationResult GetLatestResult() const = 0;
    virtual void Reset() = 0;
};

}  // namespace lightning::domain::contracts
