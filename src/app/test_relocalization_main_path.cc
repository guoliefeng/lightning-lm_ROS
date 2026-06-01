#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

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
#include "domain/contracts/state_estimator.h"
#include "domain/result/map_state.h"
#include "interfaces/localizer.h"
#include "interfaces/sensor_pipeline.h"

namespace lightning {
namespace {

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

}  // namespace
}  // namespace lightning

int main() {
    bool ok = true;
    ok &= lightning::TestCoordinatorPath();
    ok &= lightning::TestCoordinatorAccumulatingDoesNotReportSuccess();
    ok &= lightning::TestLegacyFallbackPath();
    if (!ok) {
        return EXIT_FAILURE;
    }
    std::cout << "relocalization main path smoke tests passed\n";
    return EXIT_SUCCESS;
}
