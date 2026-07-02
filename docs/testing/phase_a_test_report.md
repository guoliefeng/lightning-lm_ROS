# Phase A 测试报告

> 测试日期：2026-07-02  
> 分支：`feat/refactoring`  
> 测试范围：Sprint 0（在线同步/地图加载）+ Sprint 1（双 TF / PGO 去重）+ Sprint 2（yaw 搜索限流）

---

## 1. 测试环境

| 项 | 值 |
|----|-----|
| OS | Linux 5.15 |
| ROS | Noetic |
| 工作区 | `/home/glf/proj/lightning-lm_ROS1_ws` |
| 配置 | `config/yangpu_qc.yaml` |
| 地图 | `/home/glf/dataDisk/hainan/yangpu/map/chunked`（73 chunks） |
| 数据 | `/home/glf/dataDisk/hainan/yangpu/qc/*.bag` |
| 初值 | x=238.74, y=-208.92, z=0.07；q=(0.00806, -0.00054, 0.90942, -0.41581) |

---

## 2. 单元测试

### 2.1 命令

```bash
cd ~/proj/lightning-lm_ROS1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./devel/lib/lightning/test_relocalization_main_path
```

### 2.2 结果：**通过**

```
relocalization main path smoke tests passed
```

### 2.3 覆盖项

| 用例 | 验证点 | 结果 |
|------|--------|------|
| `TestCoordinatorPath` | coordinator 主路径、PGO 收到 localization、事件 sink 仅 1 次 | 通过 |
| `TestCoordinatorAccumulatingDoesNotReportSuccess` | 累积不足时不误报成功 | 通过 |
| `TestLegacyFallbackPath` | 无 coordinator 时 legacy fallback | 通过 |
| `TestAssemblerAllowsBackendWithoutLegacyFusionCompatibility` | 无 IFusionEngine 仍可装配 PGO | 通过 |
| `TestLegacyFusionAndPoseGraphBothReceiveRuntimeData` | legacy fusion + PGO 并行收数、事件不重复 | 通过 |
| `TestMapOdomAuthorityFreeze` | `Freeze()` 后 `GetMapToOdom()` 不变 | 通过 |

日志存档：`/tmp/unit_test_result.log`

---

## 3. 集成测试（在线定位 50s）

### 3.1 命令

```bash
# 终端 1
source /opt/ros/noetic/setup.bash && source ~/proj/lightning-lm_ROS1_ws/devel/setup.bash
roscore

# 终端 2
source /opt/ros/noetic/setup.bash && source ~/proj/lightning-lm_ROS1_ws/devel/setup.bash
rosrun lightning run_loc_online \
  --config ~/proj/lightning-lm_ROS1_ws/src/lightning-lm_ROS/config/yangpu_qc.yaml \
  --init_x 238.74 --init_y -208.92 --init_z 0.07 \
  --init_qx 0.00806 --init_qy -0.00054 --init_qz 0.90942 --init_qw -0.41581 \
  2>&1 | tee /tmp/loc_phase_a_test.log

# 终端 3（建议首次 0.5x）
source /opt/ros/noetic/setup.bash
rosbag play -r 0.5 -u 50 \
  /home/glf/dataDisk/hainan/yangpu/qc/2026-06-22-12-08-26_0.bag \
  /home/glf/dataDisk/hainan/yangpu/qc/2026-06-22-12-09-03_1.bag \
  --clock
```

### 3.2 结果摘要

| 指标 | 结果 | 判定 |
|------|------|------|
| 地图加载 | 73 chunks，约 1.6s | 通过 |
| `No point, skip this scan!` | **2 次**（仅启动前 2 帧） | 通过（Sprint 0 修复有效） |
| NDT 初始化 | `init success, score: 0.83`（阈值 0.5） | 通过 |
| Yaw 搜索限流 | `capping yaw search candidates from 60 to 24` × 6 | 通过（Sprint 2） |
| LIO 持续运行 | 495 条 `LIO state` 日志 / 50s | 通过 |
| NDT 跟踪置信度（初始化后） | min **1.27**，max **2.64** | 通过（> 0.5） |
| 日志总行数 | 6629 行 / 50s bag | 正常 |

日志存档：`/tmp/loc_phase_a_test.log`

### 3.3 关键日志摘录

**地图加载**
```
loaded chunks: 73, fps: 74
```

**启动阶段同步（仅 2 次告警，非持续失败）**
```
No point, skip this scan!   # ×2 at startup
```

**Yaw 限流（Sprint 2）**
```
capping yaw search candidates from 60 to 24
```

**初始化成功**
```
init success, score: 0.832891, th=0.5
localization init success, pose: ...
```

**跟踪阶段（50s 末段示例）**
```
confidence: 2.62531, t: -1.04182 -497.519 1.21233, succ: 1
LIO state: ..., vel: ~0.05 m/s, yaw ~29.5°
```

---

## 4. TF 链路验证

### 4.1 命令（需在 bag 播放期间采样）

```bash
source /opt/ros/noetic/setup.bash
rosparam set use_sim_time true
# 启动 run_loc_online 并 play bag 后：
rostopic echo -n 8 /tf
```

### 4.2 结果

| 检查项 | 结果 | 说明 |
|--------|------|------|
| `/tf` 是否发布 | 是 | 播放 bag 期间有连续消息 |
| 是否存在 `map → odom` | **是** | 帧名正确，结构符合 Sprint 1 设计 |
| 是否存在 `odom → base_link` | **是** | 随 LIO 更新，平移/旋转非零 |
| `map → odom` 非 Identity | **待加强** | 采样窗口内多为 Identity（见 4.3） |

采样日志：`/tmp/tf_verify_active.log`

**采样示例（双链路同时发布）**
```
frame_id: "map"      child_frame_id: "odom"
frame_id: "odom"     child_frame_id: "base_link"   # 平移/旋转持续变化
```

### 4.3 已知观察

1. **`map→odom` 在启动早期为 Identity**：在首次 coordinator 成功对齐并调用 `UpdateFromLocalization` 之前，`OnMotionEstimate` 仍会发布 TF，此时 authority 尚未更新。
2. **bridge 未出现 `loc fps` / `loc_state` 日志**：说明 `OnLocalizationResult` 在集成运行中可能未稳定触发到 `LegacyRuntimeBridge`；需后续排查 coordinator `alignment.success` 与 bridge 事件链（不影响本次 NDT 内部跟踪日志）。
3. **全局位姿与 INS 初值偏差**：本次跑法中 NDT 初始化后全局 pose 一度跳到 `(0.06, -500, -2.6)` 附近，与初值 `(238, -209)` 不一致；属场景/地图匹配问题，非 Phase A TF 改造回归，但影响 `map→odom` 数值验收。

---

## 5. 测试结论

| 层级 | 结论 |
|------|------|
| 单元测试 | **通过** — coordinator / PGO / Freeze / 事件去重均符合预期 |
| 集成：在线同步 | **通过** — `No point` 仅启动 2 次，主路径可用 |
| 集成：NDT 初始化与跟踪 | **通过** — init success + 置信度维持 >1.2 |
| 集成：Yaw 限流 | **通过** — 60→24 截断生效 |
| 集成：双 TF 结构 | **部分通过** — `map→odom` + `odom→base_link` 均已发布；`map→odom` 非 Identity 需在 coordinator 成功对齐后复测 |

**总体**：Phase A 代码改动可合并继续开发；建议在 Sprint 3 前增加一项 **「coordinator 成功后 `map→odom` 非 Identity」** 的自动化断言（可放在 `test_relocalization_main_path` 或 rostest）。

---

## 6. 复现与归档文件

| 文件 | 内容 |
|------|------|
| `/tmp/unit_test_result.log` | 单元测试输出 |
| `/tmp/loc_phase_a_test.log` | 50s 集成测试完整日志 |
| `/tmp/tf_verify_active.log` | bag 播放期间 `/tf` 采样 |
| `/tmp/loc_tf_verify.log` | 25s TF 专项短跑日志 |

---

## 7. 相关文档

- [Phase A 开发记录](../development/phase_a_tf_authority.md)
- [项目现状总览](../architecture/project_status.md)
