#pragma once

#include <string>
#include <vector>

namespace lightning::plugins::registry {

enum class PluginRole {
    kUnknown,
    kMotionEstimator,
    kLocalizer,
    kSensorCollator,
    kStateEstimator,
    kPoseGraphBackend,
    kMapStateRepository,
    kEventSink,
};

struct PluginDescriptor {
    PluginRole role = PluginRole::kUnknown;
    std::string key;
    std::string description;
};

class IPluginRegistry {
   public:
    virtual ~IPluginRegistry() = default;

    virtual bool HasPlugin(PluginRole role, const std::string& key) const = 0;
    virtual std::vector<PluginDescriptor> ListPlugins() const = 0;
    virtual std::vector<PluginDescriptor> ListPlugins(PluginRole role) const = 0;
};

}  // namespace lightning::plugins::registry
