#pragma once

// Compatibility forwarder. Stable plugin registry contracts live in domain/contracts.

#include "domain/contracts/plugin_registry.h"

namespace lightning::plugins::registry {

using PluginRole = lightning::domain::contracts::PluginRole;
using PluginDescriptor = lightning::domain::contracts::PluginDescriptor;
using IPluginRegistry = lightning::domain::contracts::IPluginRegistry;

}  // namespace lightning::plugins::registry
