# Sprint 6 开发记录：定位 UI 地图显示修复 + A201 初始化优化

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 前置：Sprint 5（`T_map_ins` 锚定）

---

## 1. 问题现象

用户在 `with_ui: true` 下运行 `run_loc_online`，Pangolin 窗口能弹出，但 **看不到地图点云**。

### 1.1 根因（已确认）

| 层级 | 现象 | 原因 |
|------|------|------|
| 地图加载 | 日志有 `loaded chunks: 73` | **地图在内存中已正确加载** |
| UI 显示 | 窗口空白 / 无灰色地图 | `LidarLoc::Init()` 向 `DynamicMapManager` 传入 **`nullptr` UI** |
| 新架构 | `LegacyRuntimeBridge` 创建了 Pangolin | 未将 `ui_` 传递给 `LidarLoc` / `LaserMapping` |
| 初值后地图块 | 仅显示原点功能点附近块 | `SetInitialPose` 后未刷新 UI 中已加载的地图块 |

结论：**不是地图没加载，而是加载后的点云没有送入 UI 渲染管线。**

---

## 2. 变更清单

### 2.1 `IUiAttachable` 接口

| 文件 | 说明 |
|------|------|
| `src/interfaces/ui_attachable.h` | 可选挂载接口 `AttachUi()` |

实现类：`LidarLocAdapter`、`LaserMappingAdapter`、`ConfigurableLocalizer`（plugin 内）。

### 2.2 地图 → UI 通路

| 组件 | 变更 |
|------|------|
| `DynamicMapManager::SetUi()` | 保存 UI 指针并刷新 |
| `DynamicMapManager::RefreshUiDisplay()` | 将 `GetStaticCloud()` / 动态图层推送到 Pangolin |
| `LidarLoc::SetUi()` | 转发到 `DynamicMapManager` |
| `LidarLoc::SetInitialPose()` | `LoadOnPose` + `RebuildTargetIfNeeded` + **`RefreshUiDisplay()`** |

### 2.3 运行时挂载（`LegacyRuntimeBridge`）

`system_root_->Start()` 成功后：

```cpp
root_impl->AttachUi(ui_);  // → TrajectoryContext → LidarLoc + LaserMapping
```

日志关键字：`runtime UI attached to localizer and motion estimator`

### 2.4 外部 INS 初值：粗搜结果被精配准毁掉

**A201 数据集暴露的问题**：

- 粗搜 NDT 在 INS 附近得分 **~1.5**（正确）
- 精配准后分数跌至 **~0.02** → 初始化失败
- 回退到错误功能点 `chunk_67 (0, -500)`，UI 显示远离车辆的地图

**修复**（`LidarLoc::YawSearch`）：精配准降分但粗搜仍超阈值时，**保留粗搜位姿**。

### 2.5 配置

| 文件 | 说明 |
|------|------|
| `config/yangpu_a201.yaml` | A201 数据集专用（`map_frame.enabled: false`，INS 初值直连） |
| `scripts/run_a201_full_test.sh` | 131 bag 全量自动化测试脚本 |

### 2.6 全量 A201（17min）暴露：LIO 发散 + PGO 刷屏

**LIO**：~2.5min 时 ivox 增至 **763k** 栅格、速度 **767 m/s**，单帧 delta 仍 <1.5m，Sprint 4 防护未触发。

**新增**（`laser_mapping` + yaml）：

| 参数 | 默认 | 行为 |
|------|------|------|
| `max_lidar_velocity_mps` | 25 | 速度超限回滚 ESKF |
| `max_ivox_grids` | 30000 | ivox 膨胀超限回滚 |

**PGO**：`ProcessLidarOdom` 对 epoch 大跳变清空 LO 队列；|Δt|<0.25s 乱序帧静默丢弃（消除 10k+ 警告）。

---

## 3. 验收命令

```bash
cd ~/proj/lightning-lm_ROS1_ws && catkin_make
./devel/lib/lightning/test_relocalization_main_path

# UI + A201（3 终端）
roscore
rosparam set use_sim_time true
./devel/lib/lightning/run_loc_online \
  --config config/yangpu_a201.yaml \
  --init_x 308.40 --init_y -150.60 --init_z -0.10 \
  --init_qx -0.001009 --init_qy 0.002793 --init_qz 0.384632 --init_qw 0.923065
rosbag play --clock -r 1.0 \
  /home/glf/dataDisk/hainan/yangpu/A201/2026-06-09-14-07-02_0.bag \
  /home/glf/dataDisk/hainan/yangpu/A201/2026-06-09-14-07-10_1.bag
```

**UI 中应看到**：

1. 灰色地图块（车辆附近）
2. 彩色当前扫描
3. 小车模型随定位移动

```bash
# 全量 131 bag（~17min，headless）
scripts/run_a201_full_test.sh
```

---

## 4. 相关文档

- [Sprint 6 UI 短测报告](../testing/sprint6_ui_and_a201_test_report.md)
- [Sprint 6 A201 全量测试报告](../testing/sprint6_a201_full_run_test_report.md)
- [Sprint 5 开发记录](sprint5_map_frame_anchor.md)
