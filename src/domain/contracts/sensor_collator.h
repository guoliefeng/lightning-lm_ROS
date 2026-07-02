#pragma once

#include <functional>

#include "domain/sensor/cloud_data.h"
#include "domain/sensor/gnss_data.h"
#include "domain/sensor/imu_data.h"
#include "domain/sensor/odometry_data.h"

namespace lightning::domain::contracts {

class ISensorCollator {
   public:
    using ImuHandler = std::function<void(const sensor::ImuData&)>;
    using CloudHandler = std::function<void(const sensor::CloudData&)>;
    using GnssHandler = std::function<void(const sensor::GnssData&)>;
    using OdometryHandler = std::function<void(const sensor::OdometryData&)>;

    virtual ~ISensorCollator() = default;

    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual void AddImuMeasurement(const sensor::ImuData& imu) = 0;
    virtual void AddCloudMeasurement(const sensor::CloudData& cloud) = 0;
    virtual void SetImuHandler(ImuHandler handler) = 0;
    virtual void SetCloudHandler(CloudHandler handler) = 0;
    virtual void Reset() = 0;

    // GNSS/轮速为可选通道，默认丢弃，保证既有实现兼容
    virtual void AddGnssMeasurement(const sensor::GnssData& gnss) {
        if (gnss_handler_) {
            gnss_handler_(gnss);
        }
    }
    virtual void AddOdometryMeasurement(const sensor::OdometryData& odom) {
        if (odometry_handler_) {
            odometry_handler_(odom);
        }
    }
    virtual void SetGnssHandler(GnssHandler handler) { gnss_handler_ = std::move(handler); }
    virtual void SetOdometryHandler(OdometryHandler handler) { odometry_handler_ = std::move(handler); }

   protected:
    GnssHandler gnss_handler_;
    OdometryHandler odometry_handler_;
};

}  // namespace lightning::domain::contracts
