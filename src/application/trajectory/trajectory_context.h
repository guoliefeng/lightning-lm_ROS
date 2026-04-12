#pragma once

// Compatibility forwarder. Stable contracts live in domain/contracts.

#include "domain/contracts/trajectory_context.h"

namespace lightning::application::trajectory {

using ITrajectoryContext = lightning::domain::contracts::ITrajectoryContext;

}  // namespace lightning::application::trajectory
