#pragma once

#include <memory>
#include <string>

#include "domain/contracts/plugin_registry.h"
#include "interfaces/fusion_engine.h"

namespace lightning::application::system {

struct LocalizationAssemblyOptions {
    std::string yaml_path;
    std::string global_map_path;
    bool online_mode = false;
    bool with_ui = false;
    std::shared_ptr<domain::contracts::IPluginRegistry> plugin_registry = nullptr;
};

struct LocalizationAssembly {
    std::shared_ptr<domain::contracts::IPluginRegistry> plugin_registry = nullptr;
    std::shared_ptr<domain::contracts::ISensorCollator> sensor_collator = nullptr;
    std::shared_ptr<domain::contracts::ISensorPipeline> sensor_pipeline = nullptr;
    std::shared_ptr<domain::contracts::IMotionEstimator> motion_estimator = nullptr;
    std::shared_ptr<domain::contracts::ILocalizer> localizer = nullptr;
    std::shared_ptr<domain::contracts::IStateEstimator> state_estimator = nullptr;
    std::shared_ptr<domain::contracts::IPoseGraphBackend> pose_graph_backend = nullptr;
    std::shared_ptr<domain::contracts::IMapStateRepository> map_state_repository = nullptr;
    std::shared_ptr<lightning::loc::IFusionEngine> legacy_fusion_engine = nullptr;
};

class SystemAssembler {
   public:
    static LocalizationAssembly AssembleLocalization(const LocalizationAssemblyOptions& options);
};

}  // namespace lightning::application::system
