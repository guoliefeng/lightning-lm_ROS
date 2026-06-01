#include "application/system/system_assembler.h"

#include <yaml-cpp/yaml.h>

#include <glog/logging.h>

#include "application/system/relocalization_coordinator.h"
#include "adapters/legacy_localizer_relocalization_adapter.h"
#include "interfaces/localizer.h"
#include "interfaces/motion_estimator.h"
#include "interfaces/sensor_pipeline.h"
#include "plugins/registry/default_plugin_registry.h"

namespace lightning::application::system {

namespace {

std::string GetComponentName(const YAML::Node& yaml, const std::string& key, const std::string& default_value) {
    const auto system = yaml["system"];
    if (!system || !system[key]) {
        return default_value;
    }
    return system[key].as<std::string>();
}

}  // namespace

LocalizationAssembly SystemAssembler::AssembleLocalization(const LocalizationAssemblyOptions& options) {
    LocalizationAssembly assembly;
    assembly.plugin_registry =
        options.plugin_registry ? options.plugin_registry : std::make_shared<plugins::registry::DefaultPluginRegistry>();

    YAML::Node yaml = YAML::LoadFile(options.yaml_path);

    const std::string sensor_collator_name =
        GetComponentName(yaml, "sensor_collator", "passthrough_sensor_collator");
    const std::string sensor_pipeline_name = GetComponentName(yaml, "sensor_pipeline", "motion_pipeline");
    const std::string motion_estimator_name = GetComponentName(yaml, "motion_estimator", "laser_mapping_adapter");
    const std::string localizer_name = GetComponentName(yaml, "localizer", "lidar_loc_adapter");
    const std::string state_estimator_name =
        GetComponentName(yaml, "state_estimator", "passthrough_state_estimator");
    const std::string pose_graph_backend_name = GetComponentName(yaml, "pose_graph_backend", "pgo_adapter");
    const std::string map_state_repository_name =
        GetComponentName(yaml, "map_state_repository", "null_map_state_repository");
    const std::string global_initializer_name =
        GetComponentName(yaml, "global_initializer", "legacy_localizer_relocalization_adapter");
    const std::string local_tracker_name =
        GetComponentName(yaml, "local_tracker", "legacy_localizer_relocalization_adapter");
    const std::string map_odom_authority_name =
        GetComponentName(yaml, "map_odom_authority", "passthrough_map_odom_authority");

    assembly.sensor_collator = assembly.plugin_registry->CreateSensorCollator(sensor_collator_name);
    assembly.motion_estimator = assembly.plugin_registry->CreateMotionEstimator(motion_estimator_name);
    assembly.sensor_pipeline = assembly.plugin_registry->CreateSensorPipeline(sensor_pipeline_name);
    assembly.localizer = assembly.plugin_registry->CreateLocalizer(localizer_name);
    assembly.state_estimator = assembly.plugin_registry->CreateStateEstimator(state_estimator_name);
    assembly.pose_graph_backend = assembly.plugin_registry->CreatePoseGraphBackend(pose_graph_backend_name);
    assembly.map_state_repository = assembly.plugin_registry->CreateMapStateRepository(map_state_repository_name);
    assembly.map_odom_authority = assembly.plugin_registry->CreateMapOdomAuthority(map_odom_authority_name);

    if (!assembly.motion_estimator || !assembly.motion_estimator->Init(options.yaml_path)) {
        LOG(ERROR) << "failed to create/init motion estimator: " << motion_estimator_name;
        return {};
    }

    if (!assembly.sensor_pipeline) {
        LOG(ERROR) << "failed to create sensor pipeline: " << sensor_pipeline_name;
        return {};
    }

    if (auto configurable_pipeline =
            std::dynamic_pointer_cast<plugins::registry::IConfigurableSensorPipeline>(assembly.sensor_pipeline)) {
        if (!configurable_pipeline->Configure(options.yaml_path, options.online_mode, assembly.motion_estimator)) {
            LOG(ERROR) << "failed to configure sensor pipeline: " << sensor_pipeline_name;
            return {};
        }
    }

    if (!assembly.localizer) {
        LOG(ERROR) << "failed to create localizer: " << localizer_name;
        return {};
    }

    if (auto configurable_localizer =
            std::dynamic_pointer_cast<plugins::registry::IConfigurableLocalizer>(assembly.localizer)) {
        if (!configurable_localizer->Configure(options.yaml_path, options.global_map_path)) {
            LOG(ERROR) << "failed to configure localizer: " << localizer_name;
            return {};
        }
    }

    std::shared_ptr<adapters::LegacyLocalizerRelocalizationAdapter> legacy_relocalization_adapter;
    const auto get_legacy_relocalization_adapter = [&]() {
        if (!legacy_relocalization_adapter) {
            legacy_relocalization_adapter =
                std::make_shared<adapters::LegacyLocalizerRelocalizationAdapter>(assembly.localizer);
        }
        return legacy_relocalization_adapter;
    };

    if (global_initializer_name == "legacy_localizer_relocalization_adapter") {
        assembly.global_initializer = get_legacy_relocalization_adapter();
    } else {
        assembly.global_initializer = assembly.plugin_registry->CreateGlobalInitializer(global_initializer_name);
    }

    if (local_tracker_name == "legacy_localizer_relocalization_adapter") {
        assembly.local_tracker = get_legacy_relocalization_adapter();
    } else {
        assembly.local_tracker = assembly.plugin_registry->CreateLocalTracker(local_tracker_name);
    }

    if (!assembly.pose_graph_backend) {
        LOG(ERROR) << "failed to create pose graph backend: " << pose_graph_backend_name;
        return {};
    }

    assembly.legacy_fusion_engine = std::dynamic_pointer_cast<lightning::loc::IFusionEngine>(assembly.pose_graph_backend);
    if (!assembly.legacy_fusion_engine) {
        LOG(ERROR) << "pose graph backend does not expose legacy fusion engine compatibility";
        return {};
    }

    if (assembly.global_initializer && assembly.local_tracker && assembly.map_odom_authority) {
        assembly.relocalization_coordinator = std::make_shared<RelocalizationCoordinator>(
            RelocalizationCoordinator::Options(), assembly.global_initializer, assembly.local_tracker,
            assembly.map_odom_authority);
    } else {
        LOG(WARNING) << "relocalization coordinator dependencies are incomplete; coordinator disabled";
    }

    return assembly;
}

}  // namespace lightning::application::system
