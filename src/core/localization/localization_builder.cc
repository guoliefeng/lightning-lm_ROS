#include "core/localization/localization_builder.h"

#include "adapters/laser_mapping_adapter.h"
#include "adapters/lidar_loc_adapter.h"
#include "adapters/pgo_adapter.h"
#include "core/lio/pointcloud_preprocess.h"
#include "io/yaml_io.h"
#include "pipelines/motion_pipeline.h"

namespace lightning::loc {

LocalizationComponents LocalizationBuilder::BuildLocalizationComponents(const std::string& yaml_path,
                                                                       const std::string& global_map_path,
                                                                       bool with_ui,
                                                                       bool online_mode,
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

    auto preprocess = std::make_shared<PointCloudPreprocess>();
    preprocess->Blind() = yaml.GetValue<double>("fasterlio", "blind");
    preprocess->TimeScale() = yaml.GetValue<double>("fasterlio", "time_scale");
    int lidar_type = yaml.GetValue<int>("fasterlio", "lidar_type");
    preprocess->NumScans() = yaml.GetValue<int>("fasterlio", "scan_line");
    preprocess->PointFilterNum() = yaml.GetValue<int>("fasterlio", "point_filter_num");

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 6) {
        preprocess->SetLidarType(LidarType::MERGED);
        LOG(INFO) << "Using merged PointCloud2 (meta_cloud)";
    } else {
        LOG(WARNING) << "unknown lidar_type";
    }

    MotionPipeline::Options motion_pipeline_options;
    motion_pipeline_options.online_mode_ = online_mode;
    motion_pipeline_options.enable_lidar_odom_skip_ = yaml.GetValue<bool>("system", "enable_lidar_odom_skip");
    motion_pipeline_options.lidar_odom_skip_num_ = yaml.GetValue<int>("system", "lidar_odom_skip_num");
    components.sensor_pipeline =
        std::make_shared<MotionPipeline>(motion_pipeline_options, components.motion_estimator, preprocess);

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
