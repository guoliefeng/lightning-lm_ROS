#pragma once

#include <functional>

#include "domain/sensor/cloud_data.h"
#include "domain/sensor/imu_data.h"

namespace lightning::application::pipelines {

class ISensorCollator {
   public:
    using ImuHandler = std::function<void(const domain::sensor::ImuData&)>;
    using CloudHandler = std::function<void(const domain::sensor::CloudData&)>;

    virtual ~ISensorCollator() = default;

    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual void AddImuMeasurement(const domain::sensor::ImuData& imu) = 0;
    virtual void AddCloudMeasurement(const domain::sensor::CloudData& cloud) = 0;
    virtual void SetImuHandler(ImuHandler handler) = 0;
    virtual void SetCloudHandler(CloudHandler handler) = 0;
    virtual void Reset() = 0;
};

}  // namespace lightning::application::pipelines
