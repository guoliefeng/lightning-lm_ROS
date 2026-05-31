#include "plugins/registry/default_plugin_registry.h"

#include <algorithm>
#include <unordered_map>

#include <glog/logging.h>

#include "adapters/global_initializer_passthrough.h"
#include "adapters/laser_mapping_adapter.h"
#include "adapters/lidar_loc_adapter.h"
#include "adapters/local_tracker_passthrough.h"
#include "adapters/map_odom_authority_passthrough.h"
#include "adapters/pgo_adapter.h"
#include "core/lio/pointcloud_preprocess.h"
#include "core/localization/lidar_loc/lidar_loc.h"
#include "io/yaml_io.h"
#include "pipelines/motion_pipeline.h"

namespace lightning::plugins::registry {

using domain::contracts::PluginDescriptor;
using domain::contracts::PluginRole;
using lightning::loc::LaserMappingAdapter;
using lightning::loc::LidarLoc;
using lightning::loc::LidarLocAdapter;
using lightning::loc::LocalizationResult;
using lightning::loc::MotionPipeline;
using lightning::loc::PGOAdapter;
using lightning::adapters::PassthroughGlobalInitializer;
using lightning::adapters::PassthroughLocalTracker;
using lightning::adapters::PassthroughMapOdomAuthority;

namespace {

class PassThroughSensorCollator : public domain::contracts::ISensorCollator {
   public:
    void Start() override { running_ = true; }
    void Stop() override { running_ = false; }

    void AddImuMeasurement(const domain::sensor::ImuData& imu) override {
        if (running_ && imu_handler_) {
            imu_handler_(imu);
        }
    }

    void AddCloudMeasurement(const domain::sensor::CloudData& cloud) override {
        if (running_ && cloud_handler_) {
            cloud_handler_(cloud);
        }
    }

    void SetImuHandler(ImuHandler handler) override { imu_handler_ = std::move(handler); }
    void SetCloudHandler(CloudHandler handler) override { cloud_handler_ = std::move(handler); }
    void Reset() override {}

   private:
    bool running_ = false;
    ImuHandler imu_handler_;
    CloudHandler cloud_handler_;
};

class PassThroughStateEstimator : public domain::contracts::IStateEstimator {
   public:
    void FeedMotionEstimate(const domain::result::MotionEstimate& motion) override {
        latest_.timestamp_s = motion.timestamp_s;
        latest_.pose = motion.pose;
        latest_.linear_velocity = motion.linear_velocity;
        latest_.angular_velocity = motion.angular_velocity;
        latest_.confidence = motion.confidence;
        latest_.valid = motion.valid;
        Publish();
    }

    void FeedLocalizationResult(const domain::result::LocalizationResult& localization) override {
        latest_.timestamp_s = localization.timestamp_s;
        latest_.pose = localization.pose;
        latest_.confidence = localization.confidence;
        latest_.valid = localization.valid;
        Publish();
    }

    void SetOutputCallback(OutputCallback callback) override { callback_ = std::move(callback); }

    domain::result::StateEstimate GetLatestEstimate() const override { return latest_; }

    void Reset() override { latest_ = domain::result::StateEstimate(); }

   private:
    void Publish() const {
        if (callback_) {
            callback_(latest_);
        }
    }

    domain::result::StateEstimate latest_;
    OutputCallback callback_;
};

class NullMapStateRepository : public domain::contracts::IMapStateRepository {
   public:
    bool Load(const std::string& map_id, domain::result::MapState& map_state) override {
        auto iter = maps_.find(map_id);
        if (iter == maps_.end()) {
            return false;
        }
        map_state = iter->second;
        return true;
    }

    void Save(const domain::result::MapState& map_state) override { maps_[map_state.map_id] = map_state; }

    bool Remove(const std::string& map_id) override { return maps_.erase(map_id) > 0; }

    std::vector<std::string> ListMapIds() const override {
        std::vector<std::string> ids;
        ids.reserve(maps_.size());
        for (const auto& [id, _] : maps_) {
            ids.emplace_back(id);
        }
        return ids;
    }

   private:
    std::unordered_map<std::string, domain::result::MapState> maps_;
};

class ConfigurableMotionPipeline : public lightning::loc::ISensorPipeline, public IConfigurableSensorPipeline {
   public:
    bool Configure(const std::string& yaml_path,
                   bool online_mode,
                   const std::shared_ptr<domain::contracts::IMotionEstimator>& motion_estimator) override {
        auto legacy_motion_estimator = std::dynamic_pointer_cast<lightning::loc::IMotionEstimator>(motion_estimator);
        if (!legacy_motion_estimator) {
            LOG(ERROR) << "sensor pipeline requires a legacy-compatible motion estimator";
            return false;
        }

        YAML_IO yaml(yaml_path);
        if (!yaml.IsOpened()) {
            LOG(ERROR) << "failed to open yaml for sensor pipeline: " << yaml_path;
            return false;
        }

        auto preprocess = std::make_shared<PointCloudPreprocess>();
        preprocess->Blind() = yaml.GetValue<double>("fasterlio", "blind");
        preprocess->TimeScale() = yaml.GetValue<double>("fasterlio", "time_scale");
        int lidar_type = yaml.GetValue<int>("fasterlio", "lidar_type");
        preprocess->NumScans() = yaml.GetValue<int>("fasterlio", "scan_line");
        preprocess->PointFilterNum() = yaml.GetValue<int>("fasterlio", "point_filter_num");
        preprocess->SetHeightROI(yaml.GetValue<float>("roi", "height_max"), yaml.GetValue<float>("roi", "height_min"));

        if (lidar_type == 1) {
            preprocess->SetLidarType(LidarType::AVIA);
        } else if (lidar_type == 2) {
            preprocess->SetLidarType(LidarType::VELO32);
        } else if (lidar_type == 3) {
            preprocess->SetLidarType(LidarType::OUST64);
        } else if (lidar_type == 4) {
            preprocess->SetLidarType(LidarType::ROBOSENSE);
        } else if (lidar_type == 6) {
            preprocess->SetLidarType(LidarType::MERGED);
        }

        MotionPipeline::Options options;
        options.online_mode_ = online_mode;
        options.enable_lidar_odom_skip_ = yaml.GetValue<bool>("system", "enable_lidar_odom_skip");
        options.lidar_odom_skip_num_ = yaml.GetValue<int>("system", "lidar_odom_skip_num");
        options.loc_on_kf_ = yaml.GetValue<bool>("lidar_loc", "loc_on_kf");

        impl_ = std::make_shared<MotionPipeline>(options, legacy_motion_estimator, preprocess);

        if (dead_reckoning_callback_) {
            impl_->SetDeadReckoningCallback(dead_reckoning_callback_);
        }
        if (lidar_odom_callback_) {
            impl_->SetLidarOdomCallback(lidar_odom_callback_);
        }
        if (keyframe_scan_callback_) {
            impl_->SetKeyframeScanCallback(keyframe_scan_callback_);
        }
        return true;
    }

    void SetDeadReckoningCallback(DeadReckoningCallback cb) override {
        dead_reckoning_callback_ = std::move(cb);
        if (impl_) {
            impl_->SetDeadReckoningCallback(dead_reckoning_callback_);
        }
    }

    void SetLidarOdomCallback(LidarOdomCallback cb) override {
        lidar_odom_callback_ = std::move(cb);
        if (impl_) {
            impl_->SetLidarOdomCallback(lidar_odom_callback_);
        }
    }

    void SetKeyframeScanCallback(KeyframeScanCallback cb) override {
        keyframe_scan_callback_ = std::move(cb);
        if (impl_) {
            impl_->SetKeyframeScanCallback(keyframe_scan_callback_);
        }
    }

    void Start() override {
        if (impl_) {
            impl_->Start();
        }
    }

    void Finish() override {
        if (impl_) {
            impl_->Finish();
        }
    }

    void ProcessIMU(IMUPtr imu) override {
        if (impl_) {
            impl_->ProcessIMU(std::move(imu));
        }
    }

    void ProcessCloud(const SensorCloudInput& cloud) override {
        if (impl_) {
            impl_->ProcessCloud(cloud);
        }
    }

   private:
    std::shared_ptr<MotionPipeline> impl_ = nullptr;
    DeadReckoningCallback dead_reckoning_callback_;
    LidarOdomCallback lidar_odom_callback_;
    KeyframeScanCallback keyframe_scan_callback_;
};

class ConfigurableLocalizer : public lightning::loc::ILocalizer, public IConfigurableLocalizer {
   public:
    bool Configure(const std::string& yaml_path, const std::string& global_map_path) override {
        YAML_IO yaml(yaml_path);
        if (!yaml.IsOpened()) {
            LOG(ERROR) << "failed to open yaml for localizer: " << yaml_path;
            return false;
        }

        LidarLoc::Options options;
        options.update_dynamic_cloud_ = yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
        options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
        options.map_option_.enable_dynamic_polygon_ = false;
        options.map_option_.map_path_ = global_map_path;

        impl_ = std::make_shared<LidarLocAdapter>(options);
        return true;
    }

    bool Init(const std::string& yaml_path) override { return impl_ ? impl_->Init(yaml_path) : false; }

    void FeedLidarOdom(const NavState& state) override {
        if (impl_) {
            impl_->FeedLidarOdom(state);
        }
    }

    void FeedDeadReckoning(const NavState& state) override {
        if (impl_) {
            impl_->FeedDeadReckoning(state);
        }
    }

    bool ProcessKeyframeScan(CloudPtr cloud) override { return impl_ ? impl_->ProcessKeyframeScan(cloud) : false; }

    void SetInitialPose(const SE3& pose) override {
        if (impl_) {
            impl_->SetInitialPose(pose);
        }
    }

    LocalizationResult GetLocalizationResult() const override {
        return impl_ ? impl_->GetLocalizationResult() : LocalizationResult();
    }

    void Finish() override {
        if (impl_) {
            impl_->Finish();
        }
    }

   private:
    std::shared_ptr<LidarLocAdapter> impl_ = nullptr;
};

}  // namespace

DefaultPluginRegistry::DefaultPluginRegistry() { RegisterDefaults(); }

void DefaultPluginRegistry::RegisterSensorCollator(const PluginDescriptor& descriptor, SensorCollatorFactory factory) {
    sensor_collator_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterSensorPipeline(const PluginDescriptor& descriptor, SensorPipelineFactory factory) {
    sensor_pipeline_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterMotionEstimator(const PluginDescriptor& descriptor, MotionEstimatorFactory factory) {
    motion_estimator_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterLocalizer(const PluginDescriptor& descriptor, LocalizerFactory factory) {
    localizer_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterStateEstimator(const PluginDescriptor& descriptor, StateEstimatorFactory factory) {
    state_estimator_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterPoseGraphBackend(const PluginDescriptor& descriptor,
                                                     PoseGraphBackendFactory factory) {
    pose_graph_backend_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterMapStateRepository(const PluginDescriptor& descriptor,
                                                       MapStateRepositoryFactory factory) {
    map_state_repository_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterGlobalInitializer(const PluginDescriptor& descriptor,
                                                      GlobalInitializerFactory factory) {
    global_initializer_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterLocalTracker(const PluginDescriptor& descriptor, LocalTrackerFactory factory) {
    local_tracker_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

void DefaultPluginRegistry::RegisterMapOdomAuthority(const PluginDescriptor& descriptor,
                                                     MapOdomAuthorityFactory factory) {
    map_odom_authority_factories_[descriptor.key] = std::move(factory);
    descriptors_.push_back(descriptor);
}

bool DefaultPluginRegistry::HasPlugin(PluginRole role, const std::string& key) const {
    return std::any_of(descriptors_.begin(), descriptors_.end(),
                       [&](const PluginDescriptor& descriptor) { return descriptor.role == role && descriptor.key == key; });
}

std::vector<PluginDescriptor> DefaultPluginRegistry::ListPlugins() const { return descriptors_; }

std::vector<PluginDescriptor> DefaultPluginRegistry::ListPlugins(PluginRole role) const {
    std::vector<PluginDescriptor> filtered;
    for (const auto& descriptor : descriptors_) {
        if (descriptor.role == role) {
            filtered.push_back(descriptor);
        }
    }
    return filtered;
}

std::shared_ptr<domain::contracts::ISensorCollator> DefaultPluginRegistry::CreateSensorCollator(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(sensor_collator_factories_),
                                std::shared_ptr<domain::contracts::ISensorCollator>>(sensor_collator_factories_, key);
}

std::shared_ptr<domain::contracts::ISensorPipeline> DefaultPluginRegistry::CreateSensorPipeline(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(sensor_pipeline_factories_),
                                std::shared_ptr<domain::contracts::ISensorPipeline>>(sensor_pipeline_factories_, key);
}

std::shared_ptr<domain::contracts::IMotionEstimator> DefaultPluginRegistry::CreateMotionEstimator(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(motion_estimator_factories_),
                                std::shared_ptr<domain::contracts::IMotionEstimator>>(motion_estimator_factories_, key);
}

std::shared_ptr<domain::contracts::ILocalizer> DefaultPluginRegistry::CreateLocalizer(const std::string& key) const {
    return CreateFromFactoryMap<decltype(localizer_factories_), std::shared_ptr<domain::contracts::ILocalizer>>(
        localizer_factories_, key);
}

std::shared_ptr<domain::contracts::IStateEstimator> DefaultPluginRegistry::CreateStateEstimator(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(state_estimator_factories_),
                                std::shared_ptr<domain::contracts::IStateEstimator>>(state_estimator_factories_, key);
}

std::shared_ptr<domain::contracts::IPoseGraphBackend> DefaultPluginRegistry::CreatePoseGraphBackend(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(pose_graph_backend_factories_),
                                std::shared_ptr<domain::contracts::IPoseGraphBackend>>(pose_graph_backend_factories_,
                                                                                       key);
}

std::shared_ptr<domain::contracts::IMapStateRepository> DefaultPluginRegistry::CreateMapStateRepository(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(map_state_repository_factories_),
                                std::shared_ptr<domain::contracts::IMapStateRepository>>(
        map_state_repository_factories_, key);
}

std::shared_ptr<domain::contracts::IGlobalInitializer> DefaultPluginRegistry::CreateGlobalInitializer(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(global_initializer_factories_),
                                std::shared_ptr<domain::contracts::IGlobalInitializer>>(
        global_initializer_factories_, key);
}

std::shared_ptr<domain::contracts::ILocalTracker> DefaultPluginRegistry::CreateLocalTracker(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(local_tracker_factories_),
                                std::shared_ptr<domain::contracts::ILocalTracker>>(local_tracker_factories_, key);
}

std::shared_ptr<domain::contracts::IMapOdomAuthority> DefaultPluginRegistry::CreateMapOdomAuthority(
    const std::string& key) const {
    return CreateFromFactoryMap<decltype(map_odom_authority_factories_),
                                std::shared_ptr<domain::contracts::IMapOdomAuthority>>(
        map_odom_authority_factories_, key);
}

void DefaultPluginRegistry::RegisterDefaults() {
    RegisterSensorCollator({PluginRole::kSensorCollator, "passthrough_sensor_collator",
                            "Minimal in-process sensor collator used before trajectory collation migration."},
                           []() { return std::make_shared<PassThroughSensorCollator>(); });

    RegisterSensorPipeline({PluginRole::kSensorPipeline, "motion_pipeline",
                            "Current motion pipeline wrapped as a registry-created component."},
                           []() { return std::make_shared<ConfigurableMotionPipeline>(); });

    RegisterMotionEstimator({PluginRole::kMotionEstimator, "laser_mapping_adapter",
                             "LaserMapping-based motion estimator adapter."},
                            []() {
                                LaserMapping::Options options;
                                options.is_in_slam_mode_ = false;
                                return std::make_shared<LaserMappingAdapter>(options);
                            });

    RegisterLocalizer({PluginRole::kLocalizer, "lidar_loc_adapter", "Lidar localization adapter with deferred options."},
                      []() { return std::make_shared<ConfigurableLocalizer>(); });

    RegisterStateEstimator({PluginRole::kStateEstimator, "passthrough_state_estimator",
                            "Minimal state estimator placeholder used before trajectory-level state fusion split."},
                           []() { return std::make_shared<PassThroughStateEstimator>(); });

    RegisterPoseGraphBackend({PluginRole::kPoseGraphBackend, "pgo_adapter",
                              "Current pose-graph backend exposed through the plugin registry."},
                             []() {
                                 auto backend = std::make_shared<lightning::loc::PGOAdapter>();
                                 backend->SetDebug(false);
                                 return std::static_pointer_cast<domain::contracts::IPoseGraphBackend>(backend);
                             });

    RegisterMapStateRepository({PluginRole::kMapStateRepository, "null_map_state_repository",
                                "In-memory placeholder repository until persistent map state storage is migrated."},
                               []() { return std::make_shared<NullMapStateRepository>(); });

    RegisterGlobalInitializer({PluginRole::kGlobalInitializer, "passthrough_global_initializer",
                               "Semantic placeholder for global scan-to-map initialization."},
                              []() { return std::make_shared<PassthroughGlobalInitializer>(); });

    RegisterLocalTracker({PluginRole::kLocalTracker, "passthrough_local_tracker",
                          "Semantic placeholder for continuous local tracking."},
                         []() { return std::make_shared<PassthroughLocalTracker>(); });

    RegisterMapOdomAuthority({PluginRole::kMapOdomAuthority, "passthrough_map_odom_authority",
                              "Single in-process authority for map-to-odom correction state."},
                             []() { return std::make_shared<PassthroughMapOdomAuthority>(); });
}

}  // namespace lightning::plugins::registry
