#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "domain/contracts/localizer.h"
#include "domain/contracts/map_state_repository.h"
#include "domain/contracts/motion_estimator.h"
#include "domain/contracts/pose_graph_backend.h"
#include "domain/contracts/sensor_collator.h"
#include "domain/contracts/sensor_pipeline.h"
#include "domain/contracts/state_estimator.h"

namespace lightning::domain::contracts {

enum class PluginRole {
    kUnknown,
    kSensorCollator,
    kSensorPipeline,
    kMotionEstimator,
    kLocalizer,
    kStateEstimator,
    kPoseGraphBackend,
    kMapStateRepository,
};

struct PluginDescriptor {
    PluginRole role = PluginRole::kUnknown;
    std::string key;
    std::string description;
};

class IPluginRegistry {
   public:
    using SensorCollatorFactory = std::function<std::shared_ptr<ISensorCollator>()>;
    using SensorPipelineFactory = std::function<std::shared_ptr<ISensorPipeline>()>;
    using MotionEstimatorFactory = std::function<std::shared_ptr<IMotionEstimator>()>;
    using LocalizerFactory = std::function<std::shared_ptr<ILocalizer>()>;
    using StateEstimatorFactory = std::function<std::shared_ptr<IStateEstimator>()>;
    using PoseGraphBackendFactory = std::function<std::shared_ptr<IPoseGraphBackend>()>;
    using MapStateRepositoryFactory = std::function<std::shared_ptr<IMapStateRepository>()>;

    virtual ~IPluginRegistry() = default;

    virtual bool HasPlugin(PluginRole role, const std::string& key) const = 0;
    virtual std::vector<PluginDescriptor> ListPlugins() const = 0;
    virtual std::vector<PluginDescriptor> ListPlugins(PluginRole role) const = 0;

    virtual std::shared_ptr<ISensorCollator> CreateSensorCollator(const std::string& key) const = 0;
    virtual std::shared_ptr<ISensorPipeline> CreateSensorPipeline(const std::string& key) const = 0;
    virtual std::shared_ptr<IMotionEstimator> CreateMotionEstimator(const std::string& key) const = 0;
    virtual std::shared_ptr<ILocalizer> CreateLocalizer(const std::string& key) const = 0;
    virtual std::shared_ptr<IStateEstimator> CreateStateEstimator(const std::string& key) const = 0;
    virtual std::shared_ptr<IPoseGraphBackend> CreatePoseGraphBackend(const std::string& key) const = 0;
    virtual std::shared_ptr<IMapStateRepository> CreateMapStateRepository(const std::string& key) const = 0;
};

}  // namespace lightning::domain::contracts
