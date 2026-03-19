//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "bridges/localization_bridge.h"
#include "core/localization/localization_runtime_factory.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_path, "./data/new_map/", "地图路径");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    loc::LocalizationRuntimeOptions options;
    options.online_mode_ = false;
    auto runtime = loc::CreateLocalizationRuntime(options);
    loc::LocalizationBridge loc_bridge(runtime);
    loc_bridge.Init(FLAGS_config, FLAGS_map_path);

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");

    rosbag
        .AddImuHandle(imu_topic,
                      [&loc_bridge](IMUPtr imu) {
                          loc_bridge.ProcessIMU(imu);
                          usleep(1000);
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc_bridge](const sensor_msgs::PointCloud2::ConstPtr& cloud) {
                                  loc_bridge.ProcessPointCloud2(cloud);
                                  usleep(1000);
                                  return true;
                              })
        .AddLivoxCloudHandle("/livox/lidar",
                             [&loc_bridge](const livox_ros_driver::CustomMsg::ConstPtr& cloud) {
                                 loc_bridge.ProcessLivoxCloud(cloud);
                                 usleep(1000);
                                 return true;
                             })
        .Go();

    Timer::PrintAll();
    loc_bridge.Finish();

    LOG(INFO) << "done";

    return 0;
}
