#ifndef LIGHTNING_IMU_FILTER_H
#define LIGHTNING_IMU_FILTER_H

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/imu.h"

namespace lightning {

class IMUFilter {
   private:
    struct Config {
        int median_window_size = 5;
        int moving_avg_window = 3;
        double rate_limit = 3.0;
        double spike_threshold = 3.0;
        bool enable_adaptive = true;
    } config_;

    std::deque<IMU> buffer_;
    std::deque<double> gyro_x_history_;
    std::deque<double> gyro_y_history_;
    std::deque<double> gyro_z_history_;

    IMU prev_filtered_;

    double gyro_mean_[3] = {0.0, 0.0, 0.0};
    double gyro_std_[3] = {0.0, 0.0, 0.0};
    int sample_count_ = 0;

   public:
    IMUFilter() { prev_filtered_.timestamp = -1.0; }

    void SetMedianWindowSize(int size) {
        if (size >= 3 && size % 2 == 1) {
            config_.median_window_size = size;
        }
    }

    void SetRateLimit(double limit) { config_.rate_limit = std::abs(limit); }

    void SetSpikeThreshold(double threshold) { config_.spike_threshold = threshold; }

    IMU Filter(const IMU& raw_data) {
        IMU filtered = raw_data;

        updateBuffer(raw_data);

        filtered.angular_velocity.x() = processAxis(raw_data.angular_velocity.x(), gyro_x_history_, 0);
        filtered.angular_velocity.y() = processAxis(raw_data.angular_velocity.y(), gyro_y_history_, 1);
        filtered.angular_velocity.z() = processAxis(raw_data.angular_velocity.z(), gyro_z_history_, 2);

        if (prev_filtered_.timestamp > 0.0) {
            const double dt = raw_data.timestamp - prev_filtered_.timestamp;
            if (dt > 0.0 && dt < 0.1) {
                filtered.angular_velocity.x() =
                    rateLimit(filtered.angular_velocity.x(), prev_filtered_.angular_velocity.x(), dt);
                filtered.angular_velocity.y() =
                    rateLimit(filtered.angular_velocity.y(), prev_filtered_.angular_velocity.y(), dt);
                filtered.angular_velocity.z() =
                    rateLimit(filtered.angular_velocity.z(), prev_filtered_.angular_velocity.z(), dt);
            }
        }

        updateStatistics(filtered);

        prev_filtered_ = filtered;
        return filtered;
    }

   private:
    double processAxis(double raw_value, std::deque<double>& history, int axis_idx) {
        double filtered = raw_value;

        if (history.size() >= static_cast<size_t>(config_.median_window_size) && sample_count_ > 100) {
            filtered = detectAndRemoveSpike(raw_value, history, axis_idx);
        }

        filtered = medianFilter(filtered, history);
        filtered = movingAverage(filtered, history);
        return filtered;
    }

    void updateBuffer(const IMU& data) {
        gyro_x_history_.push_back(data.angular_velocity.x());
        gyro_y_history_.push_back(data.angular_velocity.y());
        gyro_z_history_.push_back(data.angular_velocity.z());

        const int max_history = std::max({config_.median_window_size, config_.moving_avg_window, 10});
        while (gyro_x_history_.size() > static_cast<size_t>(max_history)) {
            gyro_x_history_.pop_front();
            gyro_y_history_.pop_front();
            gyro_z_history_.pop_front();
        }
    }

    double detectAndRemoveSpike(double value, std::deque<double>& history, int axis_idx) {
        if (history.size() < static_cast<size_t>(config_.median_window_size)) {
            return value;
        }

        std::vector<double> window(history.end() - config_.median_window_size, history.end());
        std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
        const double median = window[window.size() / 2];
        const double diff = std::abs(value - median);

        double threshold = config_.spike_threshold * gyro_std_[axis_idx];
        if (config_.enable_adaptive && gyro_std_[axis_idx] > 0.0) {
            threshold = std::max(threshold, config_.spike_threshold * 0.5);
        }

        if (diff > threshold) {
            LOG(INFO) << "find imu spike: " << diff << ", " << threshold;
            return median;
        }

        return value;
    }

    double medianFilter(double value, std::deque<double>& history) {
        if (history.size() < static_cast<size_t>(config_.median_window_size)) {
            return value;
        }

        std::vector<double> window(history.end() - config_.median_window_size, history.end());
        std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
        return window[window.size() / 2];
    }

    double movingAverage(double value, std::deque<double>& history) {
        if (history.size() < static_cast<size_t>(config_.moving_avg_window)) {
            return value;
        }

        double sum = 0.0;
        auto it = history.end() - config_.moving_avg_window;
        for (; it != history.end(); ++it) {
            sum += *it;
        }
        return sum / static_cast<double>(config_.moving_avg_window);
    }

    double rateLimit(double current, double previous, double dt) const {
        const double max_change = config_.rate_limit * dt;
        const double diff = current - previous;

        if (std::abs(diff) > max_change) {
            return previous + (diff > 0.0 ? max_change : -max_change);
        }
        return current;
    }

    void updateStatistics(const IMU& data) {
        const double alpha = 0.01;

        if (sample_count_ == 0) {
            gyro_mean_[0] = data.angular_velocity[0];
            gyro_mean_[1] = data.angular_velocity[1];
            gyro_mean_[2] = data.angular_velocity[2];
            gyro_std_[0] = 0.1;
            gyro_std_[1] = 0.1;
            gyro_std_[2] = 0.1;
        } else {
            gyro_mean_[0] = (1.0 - alpha) * gyro_mean_[0] + alpha * data.angular_velocity[0];
            gyro_mean_[1] = (1.0 - alpha) * gyro_mean_[1] + alpha * data.angular_velocity[1];
            gyro_mean_[2] = (1.0 - alpha) * gyro_mean_[2] + alpha * data.angular_velocity[2];

            gyro_std_[0] = (1.0 - alpha) * gyro_std_[0] + alpha * std::abs(data.angular_velocity[0] - gyro_mean_[0]);
            gyro_std_[1] = (1.0 - alpha) * gyro_std_[1] + alpha * std::abs(data.angular_velocity[1] - gyro_mean_[1]);
            gyro_std_[2] = (1.0 - alpha) * gyro_std_[2] + alpha * std::abs(data.angular_velocity[2] - gyro_mean_[2]);
        }

        ++sample_count_;
    }
};

}  // namespace lightning

#endif  // LIGHTNING_IMU_FILTER_H
