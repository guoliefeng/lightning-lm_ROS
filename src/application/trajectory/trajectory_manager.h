#pragma once

// Compatibility forwarder. Stable contracts live in domain/contracts.

#include "domain/contracts/trajectory_manager.h"

namespace lightning::application::trajectory {

using ITrajectoryManager = lightning::domain::contracts::ITrajectoryManager;

}  // namespace lightning::application::trajectory
