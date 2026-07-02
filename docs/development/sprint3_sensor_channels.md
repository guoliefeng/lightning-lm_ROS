# Sprint 3 开发记录：GNSS / 轮速里程计数据通道（Phase D 第一步）

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 前置：Phase A（TF 权威、PGO 去重）已合入

---

## 1. 目标

为多传感器融合（Phase D）打通数据通路：

1. domain 层新增 ROS 无关的 `GnssData` / `OdometryData` 类型。
2. `ISensorCollator` / `ITrajectoryContext` / `ISystemRoot` / `IStateEstimator` 增加 GNSS/轮速 Feed 通道（默认空实现，保证既有插件与测试兼容）。
3. ROS1 侧订阅 `NavSatFix` 与 `nav_msgs/Odometry` 并转换喂入。
4. **本 Sprint 不做融合**：`PassThroughStateEstimator` 仅记录最新值并按 100 条打点日志；真正的融合算法留待 Phase D 后续。

---

## 2. 变更清单

### 2.1 新增文件

| 文件 | 内容 |
|------|------|
| `src/domain/sensor/gnss_data.h` | `GnssData`：经纬高、`GnssFixStatus`、协方差（纯 Eigen/STL） |
| `src/domain/sensor/odometry_data.h` | `OdometryData`：位姿 + 线/角速度 + valid 标志 |
| `scripts/eval_loc_accuracy.py` | 定位精度评估脚本（TF 组合 vs 参考轨迹，支持 Umeyama 对齐/时间窗口） |

### 2.2 契约扩展（均为默认实现，不破坏既有插件）

| 契约 | 新增 |
|------|------|
| `ISensorCollator` | `AddGnssMeasurement` / `AddOdometryMeasurement` / `SetGnssHandler` / `SetOdometryHandler` |
| `ITrajectoryContext` | `FeedGnss` / `FeedOdometry`（默认丢弃） |
| `ISystemRoot` | `FeedGnss` / `FeedOdometry`（默认返回 false） |
| `IStateEstimator` | `FeedGnss` / `FeedOdometry`（默认丢弃） |
| `ILocalizationRuntime` | `FeedGnss` / `FeedOdometry`（默认丢弃） |

### 2.3 数据链路（在线定位）

```
LocSystem (订阅 gnss_topic / odom_topic)
  → LocalizationBridgeRos1::ProcessGnss / ProcessOdometry
  → sensor_conversion: NavSatFix→GnssData, Odometry→OdometryData
  → Localization::FeedGnss/FeedOdometry
  → LegacyRuntimeBridge → SystemRootImpl → TrajectoryContextImpl
  → PassThroughSensorCollator → GnssHandler/OdometryHandler
  → IStateEstimator::FeedGnss/FeedOdometry   （只记录，不融合）
```

**注意**：GNSS/轮速不进 `MotionPipeline`（LIO 主路径不受影响）。

### 2.4 配置

`config/yangpu_qc.yaml`：

```yaml
common:
  gnss_topic: "/ins_driver/gps"       # NavSatFix
  odom_topic: "/localization/ins"     # nav_msgs/Odometry
```

两个 key 均为可选：不配置则不订阅（`LocSystem` 用 try/catch 读取）。

### 2.5 单测

`test_relocalization_main_path` 新增 `TestGnssAndOdometryChannels`：
通过 `TrajectoryContextImpl::FeedGnss/FeedOdometry` 喂入数据，断言 `IStateEstimator` 收到且载荷保真。

---

## 3. 一次被回退的优化：yaw 搜索候选数上限

Phase A Sprint 2 曾把 `YawSearch` 候选数从 60 截断到 24。**本轮实测证明该优化有害，已回退**：

- 60 步 / 360° = 6° 分辨率；截到 24 步 = 15° 分辨率。
- NDT 精配准只有 4 次迭代，15° 初始角误差经常无法收敛，
  导致外部 INS 初值初始化失败，回退到错误的功能点。
- 正确做法（后续）：粗搜整帧降采样点云、或分层 yaw 搜索，而不是砍分辨率。

`lidar_loc.cc` 中已留注释说明原因，防止再次误改。

---

## 4. 待办（Phase D 后续）

1. 真正的 `IStateEstimator` 融合实现（GNSS + 轮速 + LIO + NDT 的 ESKF/滑窗）。
2. GNSS 经纬高 → 地图局部坐标的投影模块（需要地图原点锚定，依赖 Phase C MapSession）。
3. `RelocalizationCoordinator` 用 GNSS 位置约束缩小全局初始化搜索域。
4. 轮速静止检测接入 `MotionEstimate.stationary`。

---

## 5. 相关文档

- [Phase A 开发记录](phase_a_tf_authority.md)
- [Sprint 3 测试报告（含精度）](../testing/sprint3_accuracy_test_report.md)
- [项目现状总览](../architecture/project_status.md)
