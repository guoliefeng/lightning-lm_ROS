#include "core/localization/localization_builder.h"

#include "application/system/system_assembler.h"

namespace lightning::loc {

LocalizationComponents LocalizationBuilder::BuildLocalizationComponents(const std::string& yaml_path,
                                                                       const std::string& global_map_path,
                                                                       bool with_ui,
                                                                       bool online_mode,
                                                                       std::shared_ptr<ui::PangolinWindow> ui) {
    application::system::LocalizationAssemblyOptions options;
    options.yaml_path = yaml_path;
    options.global_map_path = global_map_path;
    options.online_mode = online_mode;
    options.with_ui = with_ui;

    auto assembly = application::system::SystemAssembler::AssembleLocalization(options);
    if (!assembly.motion_estimator || !assembly.sensor_pipeline || !assembly.localizer || !assembly.legacy_fusion_engine) {
        return {};
    }

    LocalizationComponents components;
    components.motion_estimator = assembly.motion_estimator;
    components.sensor_pipeline = assembly.sensor_pipeline;
    components.localizer = assembly.localizer;
    components.fusion_engine = assembly.legacy_fusion_engine;

    if (with_ui && ui) {
        components.localizer->SetUI(ui);
    }
    components.localizer->Init(yaml_path);

    return components;
}

}  // namespace lightning::loc
