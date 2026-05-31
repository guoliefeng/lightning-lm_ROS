#pragma once

#include <memory>
#include <string>

#include "core/localization/localization_result.h"
#include "interfaces/localization_runtime.h"

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::application::trajectory::legacy {
class LegacyCloudConverter;
}

namespace lightning::domain::result {
struct LocalizationResult;
struct StateEstimate;
}  // namespace lightning::domain::result

namespace lightning::domain::sensor {
struct CloudData;
}  // namespace lightning::domain::sensor

namespace lightning::domain::geometry {
struct Pose3;
}  // namespace lightning::domain::geometry

namespace lightning::domain::contracts {
class IEventSink;
class ISystemRoot;
}  // namespace lightning::domain::contracts

namespace lightning::application::system {

class LegacyRuntimeBridge {
   public:
    using TFCallback = loc::ILocalizationRuntime::TFCallback;
    using LocStateCallback = loc::ILocalizationRuntime::LocStateCallback;

    LegacyRuntimeBridge(std::shared_ptr<domain::contracts::ISystemRoot> system_root,
                        std::string trajectory_id = "default");
    ~LegacyRuntimeBridge();

    bool Init(const std::string& yaml_path, std::shared_ptr<domain::contracts::ISystemRoot> root);
    bool Init(const std::string& yaml_path);
    void Finish();

    void FeedLegacyImu(const IMUPtr& imu);
    void FeedLegacyCloud(const SensorCloudInput& cloud);
    void SetInitialPose(const SE3& pose);
    void SetTFCallback(TFCallback callback);
    void SetLocStateCallback(LocStateCallback callback);
    void SetLegacyOutputHandlers(TFCallback tf_callback, LocStateCallback loc_state_callback);
    void AttachUi(std::shared_ptr<ui::PangolinWindow> ui);
    std::shared_ptr<domain::contracts::IEventSink> CreateEventSink();

   private:
    class BridgeEventSink;

    void HandleLocalizationResult(const domain::result::LocalizationResult& result);
    void HandleStateEstimate(const domain::result::StateEstimate& estimate);
    void HandleCloudInWorld(const domain::sensor::CloudData& cloud, const domain::geometry::Pose3& pose);

    std::shared_ptr<domain::contracts::ISystemRoot> system_root_ = nullptr;
    std::string trajectory_id_;
    std::unique_ptr<trajectory::legacy::LegacyCloudConverter> cloud_converter_;
    std::shared_ptr<BridgeEventSink> event_sink_;
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
    bool owns_ui_ = false;

    TFCallback tf_callback_;
    LocStateCallback loc_state_callback_;
    loc::LocalizationResult latest_localization_result_;

    double last_imu_time_ = 0.0;
    double last_cloud_time_ = 0.0;
    bool initialized_ = false;
};

}  // namespace lightning::application::system
