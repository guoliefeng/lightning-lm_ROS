#pragma once

namespace lightning::loc {
class IMotionEstimator;
}

namespace lightning::domain::contracts {

// Temporary stable entry point over the legacy motion estimator contract.
using IMotionEstimator = lightning::loc::IMotionEstimator;

}  // namespace lightning::domain::contracts
