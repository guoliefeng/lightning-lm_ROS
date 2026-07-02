# Sprint 4 开发记录：初始化校验 + LIO 退化防护 + 事件链修复

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 前置：Sprint 3（GNSS/轮速通道）已合入；Sprint 3 测试暴露 LIO 发散与初始化误接受问题

---

## 1. 目标

针对 [Sprint 3 测试报告](../testing/sprint3_accuracy_test_report.md) 中问题 2、3：

1. **外部 INS 初值初始化**：窄 yaw 搜索 + 航向一致性校验，拒绝低分但 yaw 偏差大的解。
2. **LIO 退化防护**：单帧 ESKF 更新步长过大或观测崩溃时回滚，避免速度爆炸。
3. **coordinator → bridge 事件链**：修复 `InitWithFP` 未置 `valid_` 导致 `OnLocalizationResult` / `map→odom` TF 不发布。

---

## 2. 变更清单

### 2.1 `LidarLoc` — 外部初值初始化

| 配置项 | 默认 | 说明 |
|--------|------|------|
| `max_init_yaw_diff_deg` | 45° | 外部初值 yaw 与 NDT 结果最大允许差 |
| `external_pose_yaw_search_range` | 30° | 外部初值时 yaw 搜索半角（替代全局 180°） |

**逻辑**（`InitWithFP(..., is_external_pose=true)`）：

1. 临时将 `grid_search_angle_range` 缩至 ±30°。
2. `YawSearch` 成功后，比较估计 yaw 与 INS hint；超限则 **拒绝初始化**。
3. 功能点初始化路径不变（全角搜索、不做 yaw 校验）。

### 2.2 `LaserMapping` — 退化帧拒绝

| 配置项 | 默认 | 说明 |
|--------|------|------|
| `min_effect_feat_surf` | 80 | 极低有效特征阈值 |
| `max_lidar_frame_trans_m` | 1.5 | 单帧平移更新上限 |
| `max_lidar_frame_rot_deg` | 8.0 | 单帧旋转更新上限 |

**判定**（满足任一即回滚 `kf_` 到更新前状态）：

- `delta_trans > max_lidar_frame_trans_m`
- `delta_rot > max_lidar_frame_rot_deg`
- `effect_feat_surf < min_effect_feat_surf` **且** `delta_trans > 0.5 m`

### 2.3 事件链修复 — `valid_` 标志

`InitWithFP` 成功时补充：

```cpp
localization_result_.valid_ = true;
```

**影响**：`LegacyLocalizerRelocalizationAdapter` 的 `alignment.success` 为 true → coordinator 更新 `IMapOdomAuthority` → `HandleLocalizationResult` → bridge 出现 `loc fps` / `loc_state`，并发布 `map→odom`。

### 2.4 `LegacyRuntimeBridge` — TF 发布时机

- 新增 `map_to_odom_ready_`：仅在收到有效 localization 事件后发布 `map→odom`。
- `odom→base_link` 仍随 LIO/DR 高频更新。

---

## 3. 与 Sprint 3 的关系

| Sprint 3 问题 | Sprint 4 处理 |
|---------------|---------------|
| 低分 yaw 错解被接受 | 窄搜索 + `max_init_yaw_diff_deg` |
| LIO effect 崩 + 大步长更新 | 退化帧回滚 |
| bridge 无 `loc fps` | `valid_` 修复 |
| 地图系 ≠ INS 系 | **未解决**（依赖 Phase C `T_map_ins`） |

---

## 4. 验收命令

```bash
cd ~/proj/lightning-lm_ROS1_ws && catkin_make
./devel/lib/lightning/test_relocalization_main_path

# 集成 + 精度（60s, 0.5x）
# 见 docs/testing/sprint4_accuracy_test_report.md
```

---

## 5. 相关文档

- [Sprint 4 测试报告](../testing/sprint4_accuracy_test_report.md)
- [Sprint 3 测试报告](../testing/sprint3_accuracy_test_report.md)
