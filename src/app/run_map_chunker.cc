#include <gflags/gflags.h>
#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "common/point_def.h"
#include "core/maps/tiled_map.h"

DEFINE_string(input_pcd, "", "输入global.pcd路径");
DEFINE_string(output_dir, "", "输出目录，默认与输入PCD同目录");
DEFINE_double(chunk_size, 100.0, "分块大小（米）");
DEFINE_double(voxel_size_in_chunk, 0.1, "每个分块内部体素降采样大小（米）");
DEFINE_double(map_resolution, 0.05, "2D map分辨率（米）");
DEFINE_double(min_obstacle_height, 0.2, "高于地面该高度的点参与2D障碍投影（米）");
DEFINE_double(max_obstacle_height, 2.5, "高于地面最大障碍高度（米）");

namespace lightning {
namespace {

namespace fs = std::filesystem;

bool IsAllDigits(const std::string& s) {
    if (s.empty()) {
        return false;
    }

    for (const unsigned char c : s) {
        if (!std::isdigit(c)) {
            return false;
        }
    }

    return true;
}

bool IsStaticChunkName(const std::string& name) {
    if (name.size() <= 4 || name.substr(name.size() - 4) != ".pcd") {
        return false;
    }

    return IsAllDigits(name.substr(0, name.size() - 4));
}

bool IsDynamicChunkName(const std::string& name) {
    constexpr const char* kSuffix = "_dyn.pcd";
    constexpr size_t kSuffixLen = 8;
    if (name.size() <= kSuffixLen || name.substr(name.size() - kSuffixLen) != kSuffix) {
        return false;
    }

    return IsAllDigits(name.substr(0, name.size() - kSuffixLen));
}

void CleanupOutputDir(const fs::path& output_dir, const fs::path& input_file) {
    const fs::path input_abs = fs::absolute(input_file).lexically_normal();

    if (!fs::exists(output_dir)) {
        return;
    }

    for (const auto& entry : fs::directory_iterator(output_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const fs::path current_abs = fs::absolute(entry.path()).lexically_normal();
        if (current_abs == input_abs) {
            continue;
        }

        const std::string filename = entry.path().filename().string();
        const bool should_remove =
            filename == "index.txt" || filename == "map.pgm" || filename == "map.yaml" ||
            IsStaticChunkName(filename) || IsDynamicChunkName(filename);

        if (should_remove) {
            fs::remove(entry.path());
        }
    }
}

bool SaveProjected2DMap(const CloudPtr& cloud,
                        const fs::path& output_dir,
                        double resolution,
                        double min_obstacle_height,
                        double max_obstacle_height) {
    if (cloud == nullptr || cloud->empty()) {
        LOG(ERROR) << "cloud is empty, cannot save map.pgm/map.yaml";
        return false;
    }

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();

    for (const auto& pt : cloud->points) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
        min_z = std::min(min_z, pt.z);
    }

    if (!(max_x > min_x) || !(max_y > min_y)) {
        LOG(ERROR) << "invalid XY bounds in input pcd";
        return false;
    }

    const double width_d = std::ceil((max_x - min_x) / resolution) + 1.0;
    const double height_d = std::ceil((max_y - min_y) / resolution) + 1.0;

    const int width = static_cast<int>(width_d);
    const int height = static_cast<int>(height_d);

    if (width <= 0 || height <= 0) {
        LOG(ERROR) << "invalid map size: " << width << "x" << height;
        return false;
    }

    const double floor_z = static_cast<double>(min_z);
    const double z_low = floor_z + min_obstacle_height;
    const double z_high = floor_z + max_obstacle_height;

    size_t filtered_points = 0;
    for (const auto& pt : cloud->points) {
        const double z = static_cast<double>(pt.z);
        if (z >= z_low && z <= z_high) {
            ++filtered_points;
        }
    }

    const bool use_height_filter = filtered_points > 0;
    if (!use_height_filter) {
        LOG(WARNING) << "no points in height range [" << z_low << ", " << z_high
                     << "], fallback to all points for map projection.";
    }

    cv::Mat nav_image(height, width, CV_8UC1, cv::Scalar(255));

    for (const auto& pt : cloud->points) {
        if (use_height_filter) {
            const double z = static_cast<double>(pt.z);
            if (z < z_low || z > z_high) {
                continue;
            }
        }

        const int x = static_cast<int>(std::floor((static_cast<double>(pt.x) - static_cast<double>(min_x)) / resolution));
        const int y = static_cast<int>(std::floor((static_cast<double>(pt.y) - static_cast<double>(min_y)) / resolution));

        if (x < 0 || x >= width || y < 0 || y >= height) {
            continue;
        }

        nav_image.at<uchar>(height - 1 - y, x) = 0;
    }

    cv::Mat dilated;
    cv::dilate(nav_image, dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    const fs::path map_pgm = output_dir / "map.pgm";
    if (!cv::imwrite(map_pgm.string(), dilated)) {
        LOG(ERROR) << "failed to write " << map_pgm;
        return false;
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
    emitter << YAML::Key << "mode" << YAML::Value << "trinary";
    emitter << YAML::Key << "width" << YAML::Value << width;
    emitter << YAML::Key << "height" << YAML::Value << height;
    emitter << YAML::Key << "resolution" << YAML::Value << static_cast<float>(resolution);
    emitter << YAML::Key << "origin" << YAML::Value << std::vector<double>{static_cast<double>(min_x),
                                                                             static_cast<double>(min_y),
                                                                             0.0};
    emitter << YAML::Key << "negate" << YAML::Value << 0;
    emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
    emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;
    emitter << YAML::EndMap;

    const fs::path map_yaml = output_dir / "map.yaml";
    std::ofstream fout(map_yaml);
    if (!fout.is_open()) {
        LOG(ERROR) << "failed to open " << map_yaml;
        return false;
    }

    fout << emitter.c_str();
    fout.close();

    return true;
}

bool AppendChunkCenterFunctionalPoints(const fs::path& index_path, double chunk_size) {
    std::ifstream fin(index_path);
    if (!fin.is_open()) {
        LOG(ERROR) << "failed to open index file: " << index_path;
        return false;
    }

    std::string origin_line;
    if (!std::getline(fin, origin_line)) {
        LOG(ERROR) << "index file is empty: " << index_path;
        return false;
    }

    std::vector<std::string> chunk_lines;
    std::vector<std::string> fp_lines;
    bool reading_fp = false;

    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty()) {
            continue;
        }

        if (line == "# functional points") {
            reading_fp = true;
            continue;
        }

        if (reading_fp) {
            fp_lines.emplace_back(line);
        } else {
            chunk_lines.emplace_back(line);
        }
    }
    fin.close();

    std::vector<std::string> auto_fp_lines;
    auto_fp_lines.reserve(chunk_lines.size());

    for (const auto& chunk_line : chunk_lines) {
        std::stringstream ss(chunk_line);
        int id = 0;
        int gx = 0;
        int gy = 0;
        if (!(ss >> id >> gx >> gy)) {
            continue;
        }

        const double cx = static_cast<double>(gx) * chunk_size;
        const double cy = static_cast<double>(gy) * chunk_size;

        std::ostringstream os;
        os << std::setprecision(18) << "chunk_" << id << " " << cx << " " << cy << " 0 0 0 0 1";
        auto_fp_lines.emplace_back(os.str());
    }

    std::ofstream fout(index_path, std::ios::trunc);
    if (!fout.is_open()) {
        LOG(ERROR) << "failed to rewrite index file: " << index_path;
        return false;
    }

    fout << origin_line << '\n';
    for (const auto& chunk_line : chunk_lines) {
        fout << chunk_line << '\n';
    }

    fout << "# functional points\n";
    for (const auto& fp_line : fp_lines) {
        fout << fp_line << '\n';
    }
    for (const auto& fp_line : auto_fp_lines) {
        fout << fp_line << '\n';
    }

    fout.close();
    LOG(INFO) << "append chunk-center functional points: " << auto_fp_lines.size();
    return true;
}

bool ParseInputAndOutputFromArgs(int argc,
                                 char** argv,
                                 std::string* input_pcd_path,
                                 std::string* output_dir_path) {
#ifdef LM_SINGLE_INPUT_MODE
    if (argc > 3) {
        LOG(ERROR) << "usage: " << argv[0]
                   << " <Global.pcd> [output_dir] [--chunk_size=...] [--voxel_size_in_chunk=...]";
        return false;
    }

    if (!FLAGS_input_pcd.empty()) {
        *input_pcd_path = FLAGS_input_pcd;
    } else {
        if (argc < 2) {
            LOG(ERROR) << "usage: " << argv[0]
                       << " <Global.pcd> [output_dir] [--chunk_size=...] [--voxel_size_in_chunk=...]";
            return false;
        }
        *input_pcd_path = argv[1];
    }

    if (!FLAGS_output_dir.empty()) {
        *output_dir_path = FLAGS_output_dir;
    } else if (argc >= 3) {
        *output_dir_path = argv[2];
    } else {
        output_dir_path->clear();
    }
#else
    (void)argc;
    (void)argv;

    if (FLAGS_input_pcd.empty()) {
        LOG(ERROR) << "--input_pcd is required";
        return false;
    }

    *input_pcd_path = FLAGS_input_pcd;
    *output_dir_path = FLAGS_output_dir;
#endif

    return true;
}

}  // namespace
}  // namespace lightning

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning;
    namespace fs = std::filesystem;

    std::string input_pcd_path;
    std::string output_dir_path;
    if (!ParseInputAndOutputFromArgs(argc, argv, &input_pcd_path, &output_dir_path)) {
        return -1;
    }

    if (FLAGS_chunk_size <= 0.0 || FLAGS_voxel_size_in_chunk <= 0.0 || FLAGS_map_resolution <= 0.0) {
        LOG(ERROR) << "chunk/map parameters must be positive";
        return -1;
    }

    const fs::path input_pcd = fs::absolute(input_pcd_path).lexically_normal();
    if (!fs::exists(input_pcd) || !fs::is_regular_file(input_pcd)) {
        LOG(ERROR) << "invalid input pcd path: " << input_pcd;
        return -1;
    }

    fs::path output_dir;
    if (output_dir_path.empty()) {
        output_dir = input_pcd.parent_path();
    } else {
        output_dir = fs::absolute(output_dir_path).lexically_normal();
    }

    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    CloudPtr global_map(new PointCloudType);
    if (pcl::io::loadPCDFile<PointType>(input_pcd.string(), *global_map) != 0 || global_map->empty()) {
        LOG(ERROR) << "failed to load input pcd or cloud is empty: " << input_pcd;
        return -1;
    }

    LOG(INFO) << "loaded global map points: " << global_map->size();
    LOG(INFO) << "output dir: " << output_dir;

    CleanupOutputDir(output_dir, input_pcd);

    TiledMap::Options tm_options;
    tm_options.map_path_ = output_dir.string();
    tm_options.chunk_size_ = static_cast<float>(FLAGS_chunk_size);
    tm_options.inv_chunk_size_ = static_cast<float>(1.0 / FLAGS_chunk_size);
    tm_options.voxel_size_in_chunk_ = static_cast<float>(FLAGS_voxel_size_in_chunk);

    TiledMap tiled_map(tm_options);
    if (!tiled_map.ConvertFromFullPCD(global_map, SE3(), output_dir.string())) {
        LOG(ERROR) << "failed to convert global pcd to tiled map";
        return -1;
    }

    const fs::path index_path = output_dir / "index.txt";
    if (!AppendChunkCenterFunctionalPoints(index_path, FLAGS_chunk_size)) {
        LOG(ERROR) << "failed to append functional points into index.txt";
        return -1;
    }

    const fs::path global_out = output_dir / "global.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(global_out.string(), *global_map) != 0) {
        LOG(ERROR) << "failed to write global.pcd to " << global_out;
        return -1;
    }

    if (!SaveProjected2DMap(global_map,
                            output_dir,
                            FLAGS_map_resolution,
                            FLAGS_min_obstacle_height,
                            FLAGS_max_obstacle_height)) {
        LOG(ERROR) << "failed to generate map.pgm/map.yaml";
        return -1;
    }

    LOG(INFO) << "map conversion finished.";
    LOG(INFO) << "generated files: index.txt, *.pcd, global.pcd, map.pgm, map.yaml";
    return 0;
}
