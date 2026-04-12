#pragma once

// Compatibility forwarder. Stable contracts live in domain/contracts.

#include "domain/contracts/system_root.h"

namespace lightning::application::system {

using ISystemRoot = lightning::domain::contracts::ISystemRoot;

}  // namespace lightning::application::system
