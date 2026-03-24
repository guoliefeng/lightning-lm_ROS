# Lightning-LM

## 🌐 Language / 语言
- [English](#english)
- [中文](#中文)

---

## English

### Purpose
This ROS1 fork (`catkin` + Melodic/Noetic) is kept for learning and to document how I ran Lightning-LM locally. The upstream ROS2 project remains the canonical source; this document simply records my ROS1 workflow.

### Clone & Workspace Setup
```bash
cd ~/catkin_ws/src
git clone https://github.com/liefengguo/lightning-lm_ROS.git lightning
cd ..
catkin_make
source devel/setup.zsh    or bash
```

### ROS1 Usage
- Offline SLAM replay  
  ```bash
  roslaunch lightning run_offline.launch config_file:=config/default_nclt.yaml
  ```
- Online SLAM (live sensors or fast LiDAR data)  
  ```bash
  roslaunch lightning run_online.launch config_file:=config/fastliodata.yaml
  ```
- Localization only  
  ```bash
  rosrun lightning run_loc_online --config config/default_nclt.yaml
  ```
- Save current map artifacts anytime:  
  ```bash
  rosservice call /lightning/save_map "{map_id: demo}"
  ```

Outputs land inside `data/<map_id>/` (`global.pcd`, `tiles/`, optional `map.pgm/map.yaml`). TF publishes `map -> base_link`; Pangolin UI can be enabled via config.

### ROS Topics (Inputs/Outputs)
| Direction | Topic (default)        | Type                              | Description / Config Key                  |
|-----------|------------------------|-----------------------------------|-------------------------------------------|
| Input     | `/imu/data`            | `sensor_msgs/Imu`                 | IMU samples (`common.imu_topic`)          |
| Input     | `/velodyne_points`     | `sensor_msgs/PointCloud2`         | Standard LiDAR cloud (`common.lidar_topic`) |
| Input     | `/livox/lidar`         | `livox_ros_driver/CustomMsg`      | Livox packets (`common.livox_lidar_topic`) |
| Output    | `TF map -> base_link`  | `geometry_msgs/TransformStamped`  | Localization result (`LocalizationResult`) |
| Output    | Pangolin UI / logs     | -                                 | Optional visualization + glog             |
| Output    | `data/<map_id>/...`    | Files on disk                     | Saved point clouds & 2D grids             |

### Config Files & Key Parameters
- `config/default_nclt.yaml`: general template for offline/online use.
- `config/fastliodata.yaml`: lightweight config referenced by `run_online.launch`.
- Important fields:
  - `common.imu_topic`, `common.lidar_topic`, `common.livox_lidar_topic`
  - `system.map_path`, `system.map_name`, `system.with_loop_closing`, `system.with_ui`, `system.with_g2p5`
  - `fasterlio.lidar_type`, `fasterlio.point_filter_num`, `fasterlio.time_scale`
  - `lidar_loc.*` (localization filters), `g2p5.*` (grid map options)

Adjust these YAMLs before launch to match your dataset/topic layout. For deeper notes see `README_ROS1.md`; the rest of this file captures the original project description for reference.

---

## 中文

### 项目说明
这个 ROS1 仓库仅用于方便个人学习和记录一下，毕竟ROS1是旧时代的残党，新时代没有能承载它的船，核心算法仍来自上游 ROS2 版本。下文整理了我在 ROS1 中的运行方式，方便日后复现。
在noetic 简单测试过，可以跑通，也许还有BUG。
后续有空会加入GPS因子进行优化，maybe！
### 克隆与工作空间
```bash
cd ~/catkin_ws/src
git clone https://github.com/liefengguo/lightning-lm_ROS.git lightning
cd ..
catkin_make
source devel/setup.zsh    
```

### ROS1 使用方式
- 离线 SLAM 回放  
  ```bash
  roslaunch lightning run_offline.launch config_file:=config/default_nclt.yaml
  ```
- 在线 SLAM / 快速激光数据  
  ```bash
  roslaunch lightning run_online.launch config_file:=config/fastliodata.yaml
  ```
- 纯定位节点  
  ```bash
  rosrun lightning run_loc_online --config config/default_nclt.yaml
  ```
- 随时保存地图  
  ```bash
  rosservice call /lightning/save_map "{map_id: demo}"
  ```

所有结果都会写入 `data/<map_id>/`（`global.pcd`、`tiles/`、`map.pgm/map.yaml`）。定位默认发布 `map -> base_link` 的 TF，可通过配置打开 Pangolin UI。

### ROS 话题（输入/输出）
| 方向 | 话题（默认）              | 类型                             | 说明 / 配置键                              |
|------|---------------------------|----------------------------------|-------------------------------------------|
| 输入 | `/imu/data`               | `sensor_msgs/Imu`                | IMU 数据（`common.imu_topic`）             |
| 输入 | `/velodyne_points`        | `sensor_msgs/PointCloud2`        | 常规激光点云（`common.lidar_topic`）       |
| 输入 | `/livox/lidar`            | `livox_ros_driver/CustomMsg`     | Livox 数据（`common.livox_lidar_topic`）   |
| 输出 | `TF map -> base_link`     | `geometry_msgs/TransformStamped` | 定位结果（`LocalizationResult`）           |
| 输出 | Pangolin UI / 日志        | -                                | 可选可视化与 glog                          |
| 输出 | `data/<map_id>/...`       | 文件                              | 保存的点云与 2D 栅格                       |

### 配置文件要点
- `config/default_nclt.yaml`：通用范例，可用于离线/在线。
- `config/fastliodata.yaml`：`run_online.launch` 默认引用的精简配置。
- 关键参数：
  - `common.*`：IMU / LiDAR / Livox 话题名称
  - `system.*`：是否启用回环、栅格、UI，以及地图名/路径
  - `fasterlio.*`：激光类型、采样数、时间标定
  - `lidar_loc.*`、`g2p5.*`：定位与栅格相关选项

运行前根据数据集修改对应 YAML；更多细节可查看 `README_ROS1.md`，下方保留原始文档以供查阅。

---

## Legacy Notes (Original README)

Lightning-Speed Lidar Localization and Mapping

Features of Lightning-LM:

1. [done] Complete 3D Lidar SLAM, fast LIO front-end (AA-FasterLIO), standard
2. [done] 3D to 2D map conversion (g2p5), optional, if selected outputs real-time 2D grid map, can be saved
3. [done] Real-time loop closure detection, standard, performs back-end loop closure detection and correction if selected
4. [done] Smooth high-precision 3D Lidar localization, standard
5. [done] Dynamic loading scheme for map partitions, suitable for large-scale scenes
6. [done] Localization with separate dynamic and static layers, adaptable to dynamic scenes, selectable strategies for dynamic layer, optional, if selected saves dynamic layer map content, three strategies available (short-term, medium-term, permanent), default is permanent
7. [done] High-frequency IMU smooth output, standard, 100Hz
8. GPS geoinformation association, optional (TODO)
9. Vehicle odometry input, optional (TODO)
10. [done] Lightweight optimization library miao and incremental optimization (derived from g2o, but lighter and faster, supports incremental optimization, no need to rebuild optimization model), standard, used in both loop closure and localization
11. [done] Two verification schemes: offline and online. Offline allows breakpoint debugging with strong consistency. Online allows multi-threaded concurrency, fast processing speed, dynamic frame skipping, and low resource usage.
12. [done] High-frequency output based on extrapolator and smoother, adjustable smoothing factor
13. [done] High-performance computing: All the above features can run using less than one CPU core on the pure CPU side (online localization 0.8 cores, mapping 1.2 cores, 32-line LiDAR, without UI).

## Examples

- Mapping on the VBR campus dataset:

![](./doc/slam_vbr.gif)

- Localization on VBR

![](./doc/lm_loc_vbr_campus.gif)

- Map on VBR
   - Point Cloud

  ![](./doc/campus_vbr.png)
   - Grid Map

  ![](./doc/campus.png)

- Localization on the NCLT dataset

![](./doc/lm_loc1_nclt.gif)

## Build

### Environment

Ubuntu 22.04 or higher.

Ubuntu 20.04 should also work, but not tested.

### Dependencies

- ros2 humble or above
- Pangolin (for visualization, see thirdparty)
- OpenCV
- PCL
- yaml-cpp
- glog
- gflags
- pcl_conversions

On Ubuntu 22.04, run: ```bash ./scripts/install_dep.sh```.

### Build

Build this package with ```colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release```.

Then ```source install/setup.bash``` to use it.

### Build Results

After building, you will get the corresponding online/offline mapping and localization programs for this package. The offline programs are suitable for scenarios with offline data packets to quickly obtain mapping/localization results, while the online programs are suitable for scenarios with actual sensors to obtain real-time results.

For example, calling the offline mapping program on the NCLT dataset:
```ros2 run lightning run_slam_offline --input_bag ~/data/NCLT/20130110/20130110.db3 --config ./config/default_nclt.yaml```

If you want to call the online version, just change the offline part to online.

## Testing on Datasets

You can directly use our converted datasets. If you need the original datasets, you need to convert them to the ros2 db3 format.

Converted dataset addresses:
- OneDrive: https://1drv.ms/f/c/1a7361d22c554503/EpDSys0bWbxDhNGDYL_O0hUBa2OnhNRvNo2Gey2id7QMQA?e=7Ui0f5
- BaiduYun: https://pan.baidu.com/s/1NEGQcYoMtd57oqog09pB6w?pwd=5v8h 提取码: 5v8h 

Original dataset addresses:

- NCLT dataset: http://robots.engin.umich.edu/nclt/
- UrbanLoco dataset: https://github.com/weisongwen/UrbanLoco
- VBR dataset: https://www.rvp-group.net/slam-dataset.html

### Mapping Test

1. Real-time mapping (real-time bag playback)
   - Start the mapping program:
     ```ros2 run lightning run_slam_online --config ./config/default_nclt.yaml```
   - Play the data bag
   - Save the map ```ros2 service call /lightning/save_map /lightning/srv/SaveMap "{map_id: new_map}"```
2. Offline mapping (traverse data, faster)
   - ```ros2 run lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag [bag_file]```
   - It will automatically save to the data/new_map directory after finishing.
3. Viewing the map
   - View the full map: ```pcl_viewer ./data/new_map/global.pcd```
   - The actual map is stored in blocks, global.pcd is only for displaying the result.
   - map.pgm stores the 2D grid map information.
   - Note that during the localization program run or upon exit, results for dynamic layers might also be stored in the same directory, so there might be more files.

### Localization Test

1. Real-time localization
   - Write the map path to `system.map_path` in the yaml file, default is `new_map` (consistent with the mapping default).
   - Place the vehicle at the mapping starting point.
   - Start the localization program:
     ```ros2 run lightning run_loc_online --config ./config/default_nclt.yaml```
   - Play the bag or input sensor data.
2. Offline localization
   - ```ros2 run lightning run_loc_offline --config ./config/default_nclt.yaml --input_bag [bag_file]```
3. Receiving localization results
   - The localization program outputs TF topics at the same frequency as the IMU (50-100Hz).

### Debugging on Your Own Device

First, you need to know your LiDAR type and set the corresponding `fasterlio.lidar_type`. Set it to 1 for Livox series, 2 for Velodyne, 3 for Ouster.
If it's not one of the above types, you can refer to the Velodyne setup method.

A simpler way is to first record a ros2 bag, get offline mapping and localization working, and then debug the online situation.

You usually need to modify `common.lidar_topic` and `common.imu_topic` to set the LiDAR and IMU topics.

The IMU and LiDAR extrinsic parameters can default to zero; we are not sensitive to them.

The `fasterlio.time_scale` related to timestamps is sensitive. You should pay attention to whether the LiDAR point cloud has timestamps for each point and if they are calculated correctly. This code is in `core/lio/pointcloud_preprocess`.

Refer to the next section for other parameter adjustments.

### Fine-tuning Lightning-LM

You can fine-tune Lightning by modifying the configuration file, turning some features on or off. Common configuration items include:

- `system.with_loop_closing` Whether loop closure detection is needed
- `system.with_ui` Whether 3D UI is needed
- `system.with_2dui` Whether 2D UI is needed
- `system.with_g2p5` Whether grid map is needed
- `system.map_path` Storage path for the map
- `fasterlio.point_filter_num` Point sampling number. Increasing this results in fewer points, faster computation, but not recommended to set above 10.
- `g2p5.esti_floor` Whether g2p5 needs to dynamically estimate ground parameters. If the LiDAR rotates horizontally and the height is constant, you can turn this option off.
- `g2p5.grid_map_resolution` Resolution of the grid map

### TODO

- [done] UI displays trajectory after loop closure
- [done] Grid map saved in ROS-compatible format
- [done] Check if grid map resolution values are normal
- Force 2D output
- Additional convenience features (turn localization on/off, reinitialize, specify location, etc.)

## Miscellaneous

1. Converting ros1 data to ros2
   Install ``` pip install -i https://pypi.tuna.tsinghua.edu.cn/simple rosbags```

   Convert: ```rosbags-convert --src [your_ROS1_bag_file.bag] --dst [output_ROS2_bag_directory]```

---

Lightning-LM 是一个完整的激光建图+定位模块。

Lightning-LM特性：

1. [done] 完整的3D Lidar SLAM，快速的LIO前端（AA-FasterLIO），标配
2. [done] 3D至2D地图转换（g2p5），选配，选上的话会输出实时的2D栅格，可以保存
3. [done] 实时回环检测，标配，选上的话会进行后端回环检测并闭环
4. [done] 流畅的高精3D Lidar 定位，标配
5. [done] 地图分区动态加载方案，适用大场景
6. [done] 动静态图层分离定位，适配动态场景，可选择动态图层的策略，选配，选上的话会保存动态图层的地图内容，有三种策略可以选（短期、中期、永久），默认永久
7. [done] 高频率IMU平滑输出，标配，100Hz
8. GPS地理信息关联，选配 (TODO)
9. 车辆里程计输入，选配 (TODO)
10. [done] 轻量优化库miao以及增量式优化（来自g2o，但更轻更快，支持增量优化，不需要重新构建优化模型），标配，在回环、定位中均有用到
11. [done] 离线与在线两种验证方案。离线可以断点调试，一致性强。在线可以多线程并发，处理速度快，可以设置动态跳帧，占用低。
12. [done] 基于外推器和平滑器的高频率输出，平滑因子可调
13. [done] 高性能计算：以上这些特性在纯CPU端不到一个核心就可以运行（在线定位0.8个核，建图1.2个核，32线雷达，无UI情况）

## 案例

- VBR campus数据集上的建图：

![](./doc/slam_vbr.gif)

- VBR上的定位

![](./doc/lm_loc_vbr_campus.gif)

- VBR上的地图
   - 点云

  ![](./doc/campus_vbr.png)
   - 栅格
   
  ![](./doc/campus.png)

- NCLT 数据集上的定位

![](./doc/lm_loc1_nclt.gif)

## 编译

### 环境

Ubuntu 22.04 或更高版本。

Ubuntu 20.04 应该也可行，未测试。

### 依赖

- ros2 humble 及以上
- Pangolin（用于可视化，见thirdparty）
- OpenCV
- PCL
- yaml-cpp
- glog
- gflags
- pcl_conversions

在Ubuntu 22.04上，执行：```bash ./scripts/install_dep.sh```即可。

### 编译

```colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release```本包即可。

然后```source install/setup.bash```即可使用。

### 编译结果

编译后，会得到本包对应的在线/离线建图程序与定位程序。离线程序适用于存在离线数据包，快速得到建图/定位结果的方案，在线程序则适用于有实际传感器，得到实时结果的方案。

例如：在NCLT数据集上调用离线建图程序:
```ros2 run lightning run_slam_offline --input_bag ~/data/NCLT/20130110/20130110.db3 --config ./config/default_nclt.yaml```

如果希望调用在线的版本，则将offline部分改成online即可。

## 在数据集上测试

您可以直接使用我们转换完的数据集。如果需要原始的数据集，您需要将它们转换到ros2的db3格式。

转换后的数据集地址：
- OneDrive: https://1drv.ms/f/c/1a7361d22c554503/EpDSys0bWbxDhNGDYL_O0hUBa2OnhNRvNo2Gey2id7QMQA?e=7Ui0f5
- BaiduYun: https://pan.baidu.com/s/1NEGQcYoMtd57oqog09pB6w?pwd=5v8h 提取码: 5v8h 

原始数据集地址：

- NCLT 数据集：http://robots.engin.umich.edu/nclt/
- UrbanLoco 数据集： https://github.com/weisongwen/UrbanLoco
- VBR 数据集：https://www.rvp-group.net/slam-dataset.html

### 建图测试

1. 实时建图（实时播包）
    - 启动建图程序:
      ```ros2 run lightning run_slam_online --config ./config/default_nclt.yaml```
    - 播放数据包
    - 保存地图 ```ros2 service call /lightning/save_map /lightning/srv/SaveMap "{map_id: new_map}"```
2. 离线建图（遍历跑数据，更快一些）
    - ```ros2 run lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag 数据包```
    - 结束后会自动保存至data/new_map目录下
3. 查看地图
    - 查看完整地图：```pcl_viewer ./data/new_map/global.pcd```
    - 实际地图是分块存储的，global.pcd仅用于显示结果
    - map.pgm存储了2D栅格地图信息
    - 请注意，在定位程序运行过程中或退出时，也可能在同目录存储动态图层的结果，所以文件可能会有更多。

### 定位测试

1. 实时定位
    - 将地图路径写到yaml中的 system-map_path 下，默认是new_map（和建图默认一致)
    - 将车放在建图起点处
    - 启动定位程序：
      ```ros2 run lightning run_loc_online --config ./config/default_nclt.yaml```
    - 播包或者输入传感器数据即可

2. 离线定位
    - ```ros2 run lightning run_loc_offline --config ./config/default_nclt.yaml --input_bag 数据包```

3. 接收定位结果
    - 定位程序输出与IMU同频的TF话题（50-100Hz）

### 在您自己的设备上调试

首先您需要知道自己的雷达类型，设置对应的fasterlio.lidar_type类型。livox系列的配置成1，Velodyne的设置成2,ouster设置成3.
如果不在以上种类，可以参考velodyne的设置方式。

比较简单的方式是先录一个ros2的数据包，将离线的建图、定位调通后，再去调试在线的情况。

您通常需要修改common.lidar_topic和common.imu_topic来设置雷达与imu的话题。

imu和雷达外参默认为零就好，我们对这个不敏感。

时间戳相关的fasterlio.time_scale是敏感的。您最好关注一下雷达点云是否带有每个点的时间戳，以及它们是否计算正确。这些代码在core/lio/pointcloud_preprocess里.

其他参数调整参考下一节。

### 对lightning-lm进行微调

您可以在配置文件中对lightning进行微调，打开或者关闭一些功能。常见的配置项有：

- system.with_loop_closing 是否需要回环检测
- system.with_ui 是否需要3DUI
- system.with_2dui 是否需要2DUI
- system.with_g2p5 是否需要栅格地图
- system.map_path 地图的存储路径
- fasterlio.point_filter_num 点的采样数。调大后点数会少一些，计算更快，但不建议调到10以上。
- g2p5.esti_floor g2p5是否需要动态估计地面参数。如果雷达水平旋转且高度不变，可以关闭此选项.
- g2p5.grid_map_resolution 栅格地图的分辨率

### TODO

- [done] UI显示闭环后轨迹
- [done] 栅格地图保存为兼容ROS的形式
- [done] 检查栅格地图的分辨率取值是否正常
- 强制2D输出
- 额外便利性功能（打开关闭定位，重新初始化，指定位置等）

## 其他

1. 将ros1数据转换至ros2
   安装 ``` pip install -i https://pypi.tuna.tsinghua.edu.cn/simple rosbags```

   转换: ```rosbags-convert --src [你的ROS1_bag文件.bag] --dst [输出ROS2bag目录]```
## 更新
### 2025.11.27

- 在LIO模块中添加了Cauchy's kernel
- 在定位模块中增加了配置： try_self_extrap，默认关闭。也就是定位模块不会根据自身外推的位姿做定位（因为定位间隔较大，车辆运动较大时不准）。
- 添加了一个livox的配置文件，因为用livox的人比较多
- 如果建图时设置了固定高度，那么定位也会使用这个地图高度（默认关闭）
## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=gaoxiang12/lightning-lm&type=date&legend=top-left)](https://www.star-history.com/#gaoxiang12/lightning-lm&type=date&legend=top-left)
