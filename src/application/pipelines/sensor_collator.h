#pragma once

// Compatibility forwarder. Stable contracts live in domain/contracts.

#include "domain/contracts/sensor_collator.h"

namespace lightning::application::pipelines {

using ISensorCollator = lightning::domain::contracts::ISensorCollator;

}  // namespace lightning::application::pipelines
