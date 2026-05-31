#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "plugins/registry/plugin_registry.h"
#include "interfaces/fusion_engine.h"

namespace lightning::plugins::registry {

using PluginDescriptor = domain::contracts::PluginDescriptor;
using PluginRole = domain::contracts::PluginRole;

class IConfigurableSensorPipeline {
   public:
    virtual ~IConfigurableSensorPipeline() = default;
    virtual bool Configure(const std::string& yaml_path,
                           bool online_mode,
                           const std::shared_ptr<domain::contracts::IMotionEstimator>& motion_estimator) = 0;
};

class IConfigurableLocalizer {
   public:
    virtual ~IConfigurableLocalizer() = default;
    virtual bool Configure(const std::string& yaml_path, const std::string& global_map_path) = 0;
};

class DefaultPluginRegistry : public IPluginRegistry {
   public:
    DefaultPluginRegistry();
    ~DefaultPluginRegistry() override = default;

    void RegisterSensorCollator(const PluginDescriptor& descriptor, SensorCollatorFactory factory);
    void RegisterSensorPipeline(const PluginDescriptor& descriptor, SensorPipelineFactory factory);
    void RegisterMotionEstimator(const PluginDescriptor& descriptor, MotionEstimatorFactory factory);
    void RegisterLocalizer(const PluginDescriptor& descriptor, LocalizerFactory factory);
    void RegisterStateEstimator(const PluginDescriptor& descriptor, StateEstimatorFactory factory);
    void RegisterPoseGraphBackend(const PluginDescriptor& descriptor, PoseGraphBackendFactory factory);
    void RegisterMapStateRepository(const PluginDescriptor& descriptor, MapStateRepositoryFactory factory);
    void RegisterGlobalInitializer(const PluginDescriptor& descriptor, GlobalInitializerFactory factory);
    void RegisterLocalTracker(const PluginDescriptor& descriptor, LocalTrackerFactory factory);
    void RegisterMapOdomAuthority(const PluginDescriptor& descriptor, MapOdomAuthorityFactory factory);

    bool HasPlugin(PluginRole role, const std::string& key) const override;
    std::vector<PluginDescriptor> ListPlugins() const override;
    std::vector<PluginDescriptor> ListPlugins(PluginRole role) const override;

    std::shared_ptr<domain::contracts::ISensorCollator> CreateSensorCollator(const std::string& key) const override;
    std::shared_ptr<domain::contracts::ISensorPipeline> CreateSensorPipeline(const std::string& key) const override;
    std::shared_ptr<domain::contracts::IMotionEstimator> CreateMotionEstimator(const std::string& key) const override;
    std::shared_ptr<domain::contracts::ILocalizer> CreateLocalizer(const std::string& key) const override;
    std::shared_ptr<domain::contracts::IStateEstimator> CreateStateEstimator(const std::string& key) const override;
    std::shared_ptr<domain::contracts::IPoseGraphBackend> CreatePoseGraphBackend(const std::string& key) const override;
    std::shared_ptr<domain::contracts::IMapStateRepository> CreateMapStateRepository(
        const std::string& key) const override;
    std::shared_ptr<domain::contracts::IGlobalInitializer> CreateGlobalInitializer(
        const std::string& key) const override;
    std::shared_ptr<domain::contracts::ILocalTracker> CreateLocalTracker(const std::string& key) const override;
    std::shared_ptr<domain::contracts::IMapOdomAuthority> CreateMapOdomAuthority(
        const std::string& key) const override;

   private:
    void RegisterDefaults();

    template <typename FactoryMap>
    static bool ContainsFactory(const FactoryMap& factories, const std::string& key) {
        return factories.find(key) != factories.end();
    }

    template <typename FactoryMap, typename ProductPtr>
    static ProductPtr CreateFromFactoryMap(const FactoryMap& factories, const std::string& key) {
        auto iter = factories.find(key);
        if (iter == factories.end()) {
            return nullptr;
        }
        return iter->second();
    }

    std::unordered_map<std::string, SensorCollatorFactory> sensor_collator_factories_;
    std::unordered_map<std::string, SensorPipelineFactory> sensor_pipeline_factories_;
    std::unordered_map<std::string, MotionEstimatorFactory> motion_estimator_factories_;
    std::unordered_map<std::string, LocalizerFactory> localizer_factories_;
    std::unordered_map<std::string, StateEstimatorFactory> state_estimator_factories_;
    std::unordered_map<std::string, PoseGraphBackendFactory> pose_graph_backend_factories_;
    std::unordered_map<std::string, MapStateRepositoryFactory> map_state_repository_factories_;
    std::unordered_map<std::string, GlobalInitializerFactory> global_initializer_factories_;
    std::unordered_map<std::string, LocalTrackerFactory> local_tracker_factories_;
    std::unordered_map<std::string, MapOdomAuthorityFactory> map_odom_authority_factories_;
    std::vector<PluginDescriptor> descriptors_;
};

}  // namespace lightning::plugins::registry
