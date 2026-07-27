//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_double(init_x, 0.0, "初始位姿 x (map)");
DEFINE_double(init_y, 0.0, "初始位姿 y (map)");
DEFINE_double(init_z, 0.0, "初始位姿 z (map)");
DEFINE_double(init_qx, 0.0, "初始姿态四元数 x");
DEFINE_double(init_qy, 0.0, "初始姿态四元数 y");
DEFINE_double(init_qz, 0.0, "初始姿态四元数 z");
DEFINE_double(init_qw, 1.0, "初始姿态四元数 w");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    ros::init(argc, argv, "lightning_loc");

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    if (!loc.NeedsExternalInitPose()) {
        /// 从命令行指定初始位姿（默认原点）
        Eigen::Quaterniond init_q(FLAGS_init_qw, FLAGS_init_qx, FLAGS_init_qy, FLAGS_init_qz);
        init_q.normalize();
        loc.SetInitPose(SE3(init_q, Vec3d(FLAGS_init_x, FLAGS_init_y, FLAGS_init_z)));
    }
    loc.Spin();

    ros::shutdown();

    return 0;
}
