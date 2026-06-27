#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "application/system/relocalization_coordinator.h"
#include "application/system/system_assembler.h"
#include "application/trajectory/trajectory_context_impl.h"
#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "common/sensor_cloud_input.h"
#include "core/localization/localization_result.h"
#include "domain/contracts/event_sink.h"
#include "domain/contracts/global_initializer.h"
#include "domain/contracts/local_tracker.h"
#include "domain/contracts/map_odom_authority.h"
#include "domain/contracts/map_state_repository.h"
#include "domain/contracts/plugin_registry.h"
#include "domain/contracts/pose_graph_backend.h"
#include "domain/contracts/sensor_collator.h"
#include "domain/contracts/state_estimator.h"
#include "domain/result/map_state.h"
#include "interfaces/fusion_engine.h"
#include "interfaces/localizer.h"
#include "interfaces/motion_estimator.h"
#include "interfaces/sensor_pipeline.h"

namespace lightning {
namespace {

class FakeSensorCollator : public domain::contracts::ISensorCollator {
   public:
    void Start() override {}
    void Stop() override {}
    void AddImuMeasurement(const domain::sensor::ImuData& imu) override {
        if (imu_handler) {
            imu_handler(imu);
        }
    }
    void AddCloudMeasurement(const domain::sensor::CloudData& cloud) override {
        if (cloud_handler) {
            cloud_handler(cloud);
        }
    }
    void SetImuHandler(ImuHandler handler) override { imu_handler = std::move(handler); }
    void SetCloudHandler(CloudHandler handler) override { cloud_handler = std::move(handler); }
    void Reset() override {}

    ImuHandler imu_handler;
    CloudHandler cloud_handler;
};

class FakeSensorPipeline : public loc::ISensorPipeline {
   public:
    void SetDeadReckoningCallback(DeadReckoningCallback cb) override { dead_reckoning_callback = std::move(cb); }
    void SetLidarOdomCallback(LidarOdomCallback cb) override { lidar_odom_callback = std::move(cb); }
    void SetKeyframeScanCallback(KeyframeScanCallback cb) override { keyframe_scan_callback = std::move(cb); }
    void Start() override {}
    void Finish() override {}
    void ProcessIMU(IMUPtr) override {}
    void ProcessCloud(const SensorCloudInput&) override {}

    DeadReckoningCallback dead_reckoning_callback;
    LidarOdomCallback lidar_odom_callback;
    KeyframeScanCallback keyframe_scan_callback;
};

class FakeMotionEstimator : public loc::IMotionEstimator {
   public:
    bool Init(const std::string&) override { return true; }
    void ProcessIMU(const IMUPtr&) override {}
    void ProcessCloud(CloudPtr) override {}
    bool Run() override { return true; }
    NavState GetLidarOdomState() const override { return NavState(); }
    NavState GetDeadReckoningState() const override { return NavState(); }
    Keyframe::Ptr GetKeyframe() const override { return nullptr; }
    CloudPtr GetUndistortedScan() const override { return nullptr; }
    CloudPtr GetProjectedCloud() const override { return nullptr; }
};

class CountingLocalizer : public loc::ILocalizer {
   public:
    bool Init(const std::string&) override { return true; }
    void FeedLidarOdom(const NavState&) override {}
    void FeedDeadReckoning(const NavState&) override {}

    bool ProcessKeyframeScan(CloudPtr cloud) override {
        ++process_count;
        result.timestamp_ = cloud ? static_cast<double>(cloud->header.stamp) * 1e-9 : 0.0;
        result.valid_ = true;
        result.status_ = loc::LocalizationStatus::GOOD;
        result.lidar_loc_valid_ = true;
        result.confidence_ = 0.9;
        return true;
    }

    void SetInitialPose(const SE3&) override {}
    loc::LocalizationResult GetLocalizationResult() const override { return result; }
    void Finish() override {}

    int process_count = 0;
    loc::LocalizationResult result;
};

class CountingPoseGraphBackend : public domain::contracts::IPoseGraphBackend {
   public:
    void FeedMotionEstimate(const domain::result::MotionEstimate& motion) override {
        ++motion_count;
        latest_motion = motion;
    }

    void FeedLocalizationResult(const domain::result::LocalizationResult& localization) override {
        ++localization_count;
        latest_localization = localization;
        if (callback) {
            callback(localization);
        }
    }

    void SetOutputCallback(OutputCallback cb) override { callback = std::move(cb); }
    domain::result::LocalizationResult GetLatestResult() const override { return latest_localization; }
    void Reset() override {
        latest_motion = domain::result::MotionEstimate();
        latest_localization = domain::result::LocalizationResult();
    }

    int motion_count = 0;
    int localization_count = 0;
    domain::result::MotionEstimate latest_motion;
    domain::result::LocalizationResult latest_localization;
    OutputCallback callback;
};

class CountingFusionEngine : public loc::IFusionEngine {
   public:
    void FeedDeadReckoning(const NavState&) override { ++dead_reckoning_count; }
    void FeedLidarOdom(const NavState&) override { ++lidar_odom_count; }
    void FeedLocalization(const loc::LocalizationResult&) override { ++localization_count; }
    void SetHighFrequencyOutputCallback(OutputCallback cb) override { callback = std::move(cb); }

    int dead_reckoning_count = 0;
    int lidar_odom_count = 0;
    int localization_count = 0;
    OutputCallback callback;
};

class CountingStateEstimator : public domain::contracts::IStateEstimator {
   public:
    void FeedMotionEstimate(const domain::result::MotionEstimate& motion) override {
        latest.timestamp_s = motion.timestamp_s;
        latest.pose = motion.pose;
        latest.valid = motion.valid;
        if (callback) {
            callback(latest);
        }
    }

    void FeedLocalizationResult(const domain::result::LocalizationResult& localization) override {
        ++localization_count;
        latest.timestamp_s = localization.timestamp_s;
        latest.pose = localization.pose;
        latest.confidence = localization.confidence;
        latest.valid = localization.valid;
        if (callback) {
            callback(latest);
        }
    }

    void SetOutputCallback(OutputCallback cb) override { callback = std::move(cb); }
    domain::result::StateEstimate GetLatestEstimate() const override { return latest; }
    void Reset() override { latest = domain::result::StateEstimate(); }

    int localization_count = 0;
    domain::result::StateEstimate latest;
    OutputCallback callback;
};

class CountingEventSink : public domain::contracts::IEventSink {
   public:
    void OnMotionEstimate(const domain::result::MotionEstimate&) override { ++motion_count; }
    void OnStateEstimate(const domain::result::StateEstimate&) override { ++state_count; }
    void OnLocalizationResult(const domain::result::LocalizationResult& result) override {
        ++localization_count;
        latest_localization = result;
    }
    void OnRelocalizationState(domain::result::RelocalizationState state) override {
        ++relocalization_state_count;
        latest_relocalization_state = state;
    }
    void OnMapState(const domain::result::MapState&) override { ++map_state_count; }
    void OnCloudInWorld(const domain::sensor::CloudData&, const domain::geometry::Pose3&) override {
        ++cloud_in_world_count;
    }

    int motion_count = 0;
    int state_count = 0;
    int localization_count = 0;
    int relocalization_state_count = 0;
    int map_state_count = 0;
    int cloud_in_world_count = 0;
    domain::result::LocalizationResult latest_localization;
    domain::result::RelocalizationState latest_relocalization_state =
        domain::result::RelocalizationState::kIdle;
};

class SuccessGlobalInitializer : public domain::contracts::IGlobalInitializer {
   public:
    bool IsReady(const domain::sensor::ScanSnapshot& snapshot) const override {
        return !snapshot.registered_scan.points.empty();
    }

    domain::result::AlignmentResult Initialize(const domain::sensor::ScanSnapshot& snapshot,
                                               const domain::result::MapState*) override {
        ++initialize_count;
        latest_snapshot = snapshot;
        domain::result::AlignmentResult result;
        result.pose = snapshot.has_odom_pose_hint ? snapshot.odom_pose_hint : domain::geometry::Pose3::Identity();
        result.confidence = 0.75;
        result.success = true;
        result.status = domain::result::AlignmentStatus::kConverged;
        return result;
    }

    void Reset() override {}

    int initialize_count = 0;
    domain::sensor::ScanSnapshot latest_snapshot;
};

class SuccessLocalTracker : public domain::contracts::ILocalTracker {
   public:
    domain::result::AlignmentResult Track(const domain::sensor::ScanSnapshot&,
                                          const domain::geometry::Pose3& initial_pose) override {
        ++track_count;
        domain::result::AlignmentResult result;
        result.pose = initial_pose;
        result.confidence = 1.0;
        result.success = true;
        result.status = domain::result::AlignmentStatus::kConverged;
        return result;
    }

    void Reset() override {}

    int track_count = 0;
};

class CountingMapOdomAuthority : public domain::contracts::IMapOdomAuthority {
   public:
    bool UpdateFromLocalization(const domain::result::LocalizationResult& localization,
                                const domain::geometry::Pose3&) override {
        if (frozen || !localization.valid) {
            return false;
        }
        ++update_count;
        map_to_odom = localization.pose;
        return true;
    }

    domain::geometry::Pose3 GetMapToOdom() const override { return map_to_odom; }
    void Reset() override {
        map_to_odom = domain::geometry::Pose3::Identity();
        frozen = false;
    }
    void Freeze() override { frozen = true; }
    void Unfreeze() override { frozen = false; }
    bool IsFrozen() const override { return frozen; }

    int update_count = 0;
    bool frozen = false;
    domain::geometry::Pose3 map_to_odom = domain::geometry::Pose3::Identity();
};

class FakeMapStateRepository : public domain::contracts::IMapStateRepository {
   public:
    bool Load(const std::string&, domain::result::MapState&) override { return false; }
    void Save(const domain::result::MapState&) override {}
    bool Remove(const std::string&) override { return false; }
    std::vector<std::string> ListMapIds() const override { return {}; }
};

class FakePluginRegistry : public domain::contracts::IPluginRegistry {
   public:
    FakePluginRegistry()
        : sensor_collator(std::make_shared<FakeSensorCollator>()),
          sensor_pipeline(std::make_shared<FakeSensorPipeline>()),
          motion_estimator(std::make_shared<FakeMotionEstimator>()),
          localizer(std::make_shared<CountingLocalizer>()),
          state_estimator(std::make_shared<CountingStateEstimator>()),
          pose_graph_backend(std::make_shared<CountingPoseGraphBackend>()),
          map_state_repository(std::make_shared<FakeMapStateRepository>()),
          map_odom_authority(std::make_shared<CountingMapOdomAuthority>()) {}

    bool HasPlugin(domain::contracts::PluginRole, const std::string&) const override { return true; }
    std::vector<domain::contracts::PluginDescriptor> ListPlugins() const override { return {}; }
    std::vector<domain::contracts::PluginDescriptor> ListPlugins(domain::contracts::PluginRole) const override {
        return {};
    }

    std::shared_ptr<domain::contracts::ISensorCollator> CreateSensorCollator(const std::string&) const override {
        return sensor_collator;
    }
    std::shared_ptr<domain::contracts::ISensorPipeline> CreateSensorPipeline(const std::string&) const override {
        return sensor_pipeline;
    }
    std::shared_ptr<domain::contracts::IMotionEstimator> CreateMotionEstimator(const std::string&) const override {
        return motion_estimator;
    }
    std::shared_ptr<domain::contracts::ILocalizer> CreateLocalizer(const std::string&) const override {
        return localizer;
    }
    std::shared_ptr<domain::contracts::IStateEstimator> CreateStateEstimator(const std::string&) const override {
        return state_estimator;
    }
    std::shared_ptr<domain::contracts::IPoseGraphBackend> CreatePoseGraphBackend(const std::string&) const override {
        return pose_graph_backend;
    }
    std::shared_ptr<domain::contracts::IMapStateRepository> CreateMapStateRepository(
        const std::string&) const override {
        return map_state_repository;
    }
    std::shared_ptr<domain::contracts::IGlobalInitializer> CreateGlobalInitializer(const std::string&) const override {
        return nullptr;
    }
    std::shared_ptr<domain::contracts::ILocalTracker> CreateLocalTracker(const std::string&) const override {
        return nullptr;
    }
    std::shared_ptr<domain::contracts::IMapOdomAuthority> CreateMapOdomAuthority(const std::string&) const override {
        return map_odom_authority;
    }

    std::shared_ptr<FakeSensorCollator> sensor_collator;
    std::shared_ptr<FakeSensorPipeline> sensor_pipeline;
    std::shared_ptr<FakeMotionEstimator> motion_estimator;
    std::shared_ptr<CountingLocalizer> localizer;
    std::shared_ptr<CountingStateEstimator> state_estimator;
    std::shared_ptr<CountingPoseGraphBackend> pose_graph_backend;
    std::shared_ptr<FakeMapStateRepository> map_state_repository;
    std::shared_ptr<CountingMapOdomAuthority> map_odom_authority;
};

CloudPtr MakeCloud() {
    CloudPtr cloud(new PointCloudType());
    cloud->header.stamp = 1230000000ULL;
    cloud->is_dense = true;

    PointType point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.intensity = 4.0f;
    point.time = 0.0;
    cloud->push_back(point);
    return cloud;
}

NavState MakeNavState(double timestamp_s) {
    NavState state;
    state.timestamp_ = timestamp_s;
    state.pose_is_ok_ = true;
    return state;
}

std::string WriteAssemblerSmokeYaml() {
    const std::string path = "/tmp/lightning_backend_decoupling_smoke.yaml";
    std::ofstream out(path);
    out << "system:\n"
        << "  sensor_collator: fake_sensor_collator\n"
        << "  sensor_pipeline: fake_sensor_pipeline\n"
        << "  motion_estimator: fake_motion_estimator\n"
        << "  localizer: fake_localizer\n"
        << "  state_estimator: fake_state_estimator\n"
        << "  pose_graph_backend: fake_pose_graph_backend\n"
        << "  map_state_repository: fake_map_state_repository\n"
        << "  map_odom_authority: fake_map_odom_authority\n";
    return path;
}

bool Check(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        return false;
    }
    return true;
}

bool TestCoordinatorPath() {
    auto pipeline = std::make_shared<FakeSensorPipeline>();
    auto localizer = std::make_shared<CountingLocalizer>();
    auto state_estimator = std::make_shared<CountingStateEstimator>();
    auto event_sink = std::make_shared<CountingEventSink>();
    auto global_initializer = std::make_shared<SuccessGlobalInitializer>();
    auto local_tracker = std::make_shared<SuccessLocalTracker>();
    auto map_odom_authority = std::make_shared<CountingMapOdomAuthority>();

    application::system::RelocalizationCoordinator::Options coordinator_options;
    coordinator_options.min_accumulated_scans = 1;
    coordinator_options.min_accumulated_points = 1;
    auto coordinator = std::make_shared<application::system::RelocalizationCoordinator>(
        coordinator_options, global_initializer, local_tracker, map_odom_authority);

    application::system::LocalizationAssembly assembly;
    assembly.sensor_pipeline = pipeline;
    assembly.localizer = localizer;
    assembly.state_estimator = state_estimator;
    assembly.relocalization_coordinator = coordinator;

    application::trajectory::TrajectoryContextImpl::Options options;
    options.id = "coordinator_path";
    application::trajectory::TrajectoryContextImpl context(options, std::move(assembly));
    context.SetEventSink(event_sink);

    pipeline->keyframe_scan_callback(MakeCloud());

    bool ok = true;
    ok &= Check(global_initializer->initialize_count == 1, "coordinator global initializer was not called");
    ok &= Check(localizer->process_count == 0, "legacy localizer was called despite coordinator availability");
    ok &= Check(state_estimator->localization_count == 1, "state estimator did not receive coordinator result");
    ok &= Check(event_sink->localization_count == 1, "event sink did not receive coordinator result");
    ok &= Check(event_sink->cloud_in_world_count == 1, "cloud-in-world visualization event was not emitted");
    ok &= Check(map_odom_authority->update_count == 1, "map->odom authority was not updated");
    ok &= Check(context.GetLatestLocalizationResult().valid, "latest localization result was not updated");
    ok &= Check(global_initializer->latest_snapshot.source_id == "coordinator_path",
                "scan snapshot source_id was not populated");
    ok &= Check(global_initializer->latest_snapshot.stamp_ns == 1230000000ULL,
                "scan snapshot stamp_ns did not preserve keyframe stamp");
    return ok;
}

bool TestCoordinatorAccumulatingDoesNotReportSuccess() {
    auto pipeline = std::make_shared<FakeSensorPipeline>();
    auto localizer = std::make_shared<CountingLocalizer>();
    auto state_estimator = std::make_shared<CountingStateEstimator>();
    auto event_sink = std::make_shared<CountingEventSink>();
    auto global_initializer = std::make_shared<SuccessGlobalInitializer>();
    auto local_tracker = std::make_shared<SuccessLocalTracker>();
    auto map_odom_authority = std::make_shared<CountingMapOdomAuthority>();

    application::system::RelocalizationCoordinator::Options coordinator_options;
    coordinator_options.min_accumulated_scans = 1;
    coordinator_options.min_accumulated_points = 2;
    auto coordinator = std::make_shared<application::system::RelocalizationCoordinator>(
        coordinator_options, global_initializer, local_tracker, map_odom_authority);

    application::system::LocalizationAssembly assembly;
    assembly.sensor_pipeline = pipeline;
    assembly.localizer = localizer;
    assembly.state_estimator = state_estimator;
    assembly.relocalization_coordinator = coordinator;

    application::trajectory::TrajectoryContextImpl::Options options;
    options.id = "accumulating_path";
    application::trajectory::TrajectoryContextImpl context(options, std::move(assembly));
    context.SetEventSink(event_sink);

    pipeline->keyframe_scan_callback(MakeCloud());

    bool ok = true;
    ok &= Check(global_initializer->initialize_count == 0, "initializer ran before enough accumulation");
    ok &= Check(localizer->process_count == 0, "legacy fallback ran while coordinator was accumulating");
    ok &= Check(state_estimator->localization_count == 0, "state estimator received false localization success");
    ok &= Check(event_sink->localization_count == 0, "event sink received false localization success");
    ok &= Check(!context.GetLatestLocalizationResult().valid, "latest localization was marked valid while accumulating");
    ok &= Check(event_sink->relocalization_state_count > 0, "accumulating state event was not emitted");
    return ok;
}

bool TestLegacyFallbackPath() {
    auto pipeline = std::make_shared<FakeSensorPipeline>();
    auto localizer = std::make_shared<CountingLocalizer>();
    auto state_estimator = std::make_shared<CountingStateEstimator>();
    auto event_sink = std::make_shared<CountingEventSink>();

    application::system::LocalizationAssembly assembly;
    assembly.sensor_pipeline = pipeline;
    assembly.localizer = localizer;
    assembly.state_estimator = state_estimator;

    application::trajectory::TrajectoryContextImpl::Options options;
    options.id = "legacy_fallback";
    application::trajectory::TrajectoryContextImpl context(options, std::move(assembly));
    context.SetEventSink(event_sink);

    pipeline->keyframe_scan_callback(MakeCloud());

    bool ok = true;
    ok &= Check(localizer->process_count == 1, "legacy localizer fallback was not called");
    ok &= Check(state_estimator->localization_count == 1, "state estimator did not receive fallback result");
    ok &= Check(event_sink->localization_count == 1, "event sink did not receive fallback result");
    ok &= Check(context.GetLatestLocalizationResult().valid, "fallback latest localization result was not updated");
    return ok;
}

bool TestAssemblerAllowsBackendWithoutLegacyFusionCompatibility() {
    auto registry = std::make_shared<FakePluginRegistry>();

    application::system::LocalizationAssemblyOptions options;
    options.yaml_path = WriteAssemblerSmokeYaml();
    options.plugin_registry = registry;

    const auto assembly = application::system::SystemAssembler::AssembleLocalization(options);

    bool ok = true;
    ok &= Check(assembly.pose_graph_backend != nullptr,
                "assembler failed to keep pose_graph_backend without IFusionEngine compatibility");
    ok &= Check(assembly.legacy_fusion_engine == nullptr,
                "assembler unexpectedly created legacy fusion engine from a backend-only pose graph");
    ok &= Check(assembly.relocalization_coordinator != nullptr,
                "assembler did not create coordinator with legacy localizer relocalization adapter");
    return ok;
}

bool TestLegacyFusionAndPoseGraphBothReceiveRuntimeData() {
    auto pipeline = std::make_shared<FakeSensorPipeline>();
    auto localizer = std::make_shared<CountingLocalizer>();
    auto state_estimator = std::make_shared<CountingStateEstimator>();
    auto pose_graph_backend = std::make_shared<CountingPoseGraphBackend>();
    auto legacy_fusion_engine = std::make_shared<CountingFusionEngine>();
    auto event_sink = std::make_shared<CountingEventSink>();

    application::system::LocalizationAssembly assembly;
    assembly.sensor_pipeline = pipeline;
    assembly.localizer = localizer;
    assembly.state_estimator = state_estimator;
    assembly.pose_graph_backend = pose_graph_backend;
    assembly.legacy_fusion_engine = legacy_fusion_engine;

    application::trajectory::TrajectoryContextImpl::Options options;
    options.id = "backend_decoupling";
    application::trajectory::TrajectoryContextImpl context(options, std::move(assembly));
    context.SetEventSink(event_sink);

    pipeline->dead_reckoning_callback(MakeNavState(1.0));
    pipeline->lidar_odom_callback(MakeNavState(2.0));
    pipeline->keyframe_scan_callback(MakeCloud());

    bool ok = true;
    ok &= Check(legacy_fusion_engine->dead_reckoning_count == 1,
                "legacy fusion did not receive dead reckoning");
    ok &= Check(legacy_fusion_engine->lidar_odom_count == 1,
                "legacy fusion did not receive lidar odometry");
    ok &= Check(legacy_fusion_engine->localization_count == 1,
                "legacy fusion did not receive localization fallback");
    ok &= Check(pose_graph_backend->motion_count == 2,
                "pose graph backend did not receive motion estimates when legacy fusion existed");
    ok &= Check(pose_graph_backend->localization_count == 1,
                "pose graph backend did not receive localization when legacy fusion existed");
    ok &= Check(event_sink->localization_count == 1,
                "pose graph output callback did not publish localization event");
    return ok;
}

}  // namespace
}  // namespace lightning

int main() {
    bool ok = true;
    ok &= lightning::TestCoordinatorPath();
    ok &= lightning::TestCoordinatorAccumulatingDoesNotReportSuccess();
    ok &= lightning::TestLegacyFallbackPath();
    ok &= lightning::TestAssemblerAllowsBackendWithoutLegacyFusionCompatibility();
    ok &= lightning::TestLegacyFusionAndPoseGraphBothReceiveRuntimeData();
    if (!ok) {
        return EXIT_FAILURE;
    }
    std::cout << "relocalization main path smoke tests passed\n";
    return EXIT_SUCCESS;
}
