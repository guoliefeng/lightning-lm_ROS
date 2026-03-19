#include "core/localization/localization_builder.h"

#include "adapters/laser_mapping_adapter.h"
#include "adapters/lidar_loc_adapter.h"
#include "adapters/pgo_adapter.h"
#include "io/yaml_io.h"

namespace lightning::loc {

LocalizationComponents LocalizationBuilder::BuildLocalizationComponents(const std::string& yaml_path,
                                                                       const std::string& global_map_path,
                                                                       bool with_ui,
                                                                       std::shared_ptr<ui::PangolinWindow> ui) {
    LocalizationComponents components;

    YAML_IO yaml(yaml_path);

    LaserMapping::Options opt_lio;
    opt_lio.is_in_slam_mode_ = false;

    components.motion_estimator = std::make_shared<LaserMappingAdapter>(opt_lio);
    if (!components.motion_estimator->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio";
        return {};
    }

    LidarLoc::Options lidar_loc_options;
    lidar_loc_options.update_dynamic_cloud_ = yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
    lidar_loc_options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
    lidar_loc_options.map_option_.enable_dynamic_polygon_ = false;
    lidar_loc_options.map_option_.map_path_ = global_map_path;
    components.localizer = std::make_shared<LidarLocAdapter>(lidar_loc_options);

    if (with_ui && ui) {
        components.localizer->SetUI(ui);
    }
    components.localizer->Init(yaml_path);

    auto fusion_engine = std::make_shared<PGOAdapter>();
    fusion_engine->SetDebug(false);
    components.fusion_engine = fusion_engine;

    return components;
}

}  // namespace lightning::loc
