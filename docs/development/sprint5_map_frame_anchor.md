# Sprint 5 开发记录：地图系锚定 `T_map_ins`

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 前置：Sprint 4（INS 初值校验 + LIO 退化防护 + 事件链修复）

---

## 1. 目标

解决 Sprint 3/4 精度评估中 **INS 参考轨迹与定位地图坐标系不一致** 的问题：

1. 在配置中声明 `T_ins_to_map`（INS/外部里程计系 → 定位地图系）。
2. 在线启动时将 `--init_x/y/z`（INS 系）变换到地图系再送入 `LidarLoc`。
3. 评估脚本对 `/localization/ins` 参考轨迹施加同一变换，使 ATE 反映真实定位误差而非坐标系常值偏移。
4. 统一 TF 发布时间基准，修复 `map→odom` 与 `odom→base_link` 时间戳混用导致评估配对失败。

---

## 2. 变更清单

### 2.1 新增 `MapFrameAnchor`

| 文件 | 说明 |
|------|------|
| `src/map_runtime/map_frame_anchor.h` | 锚定变换 API |
| `src/map_runtime/map_frame_anchor.cc` | 从 YAML 加载并变换位姿/点 |

**变换定义**：`p_map = T_ins_to_map * p_ins`（SE3 左乘）。

**YAML 模式**：

| `mode` | 配置项 | 说明 |
|--------|--------|------|
| `fixed` | `ins_to_map_translation`, `ins_to_map_rotation_rpy_deg` | 直接指定 6-DOF |
| `anchor_pair` | `ins_anchor`, `map_anchor`（x/y/z + qx/qy/qz/qw） | 由标定点对求 `T_map_anchor * T_ins_anchor⁻¹` |

### 2.2 `run_loc_online` — 初值坐标变换

- 新增 `--init_in_ins_frame`（默认 `true`）。
- 当 `map_frame.enabled=true` 且 `init_in_ins_frame=true` 时，对 CLI 初值施加 `TransformInsToMap()` 后调用 `SetInitPose()`。
- 日志打印 INS 系与地图系两套初值，便于核对标定。

### 2.3 `config/yangpu_qc.yaml` — yangpu 标定

当前采用 **单次成功 NDT 定位结果** 与 **bag 首帧 INS** 的平移差作为粗略锚定（`mode: fixed`）：

```yaml
map_frame:
  enabled: true
  mode: fixed
  ins_to_map_translation: [-238.5073, -191.3690, 0.8848]
  ins_to_map_rotation_rpy_deg: [0.0, 0.0, 0.0]
```

> 注：平移锚定后平面 RMSE 从 Sprint 3 的数百米降至约 15–25 m；**yaw 偏置尚未标定**（评估中 yaw 误差仍约 60°+），后续可用 `anchor_pair` 或最小二乘多点标定。

### 2.4 `scripts/eval_loc_accuracy.py` — 参考轨迹变换

- `--config <yaml>`：读取 `map_frame` 段，对 INS 参考施加 `T_ins_to_map`。
- `--ins-to-map-trans dx,dy,dz` / `--ins-to-map-yaw deg`：命令行覆盖。
- TF 配对改为 **最近时间戳** 查找（`max_dt=50ms`），容忍 `map→odom` 与 `odom→base_link` 非严格同 stamp。

### 2.5 `LegacyRuntimeBridge` — TF 时间戳统一

`PublishSplitTf()` 改为使用 `ros::Time::now()`（仿真时来自 `/clock`）作为两条 TF 的 `header.stamp`，避免：

- `map→odom` 使用传感器 epoch（~1.78e9）
- `odom→base_link` 混用回放 clock（~0.01）

导致评估脚本无法正确组合位姿链。

---

## 3. 与 Sprint 4 的关系

| Sprint 4 遗留 | Sprint 5 处理 |
|---------------|---------------|
| 地图系 ≠ INS 系，Umeyama 对齐仍数百米 | 配置化 `T_ins_to_map` + 评估脚本同步变换 |
| 外部 INS 初值直接进地图系易偏 | `run_loc_online` 自动 INS→map 变换 |
| TF 评估配对不稳 | ROS clock 统一 stamp + 最近邻配对 |

**未解决**（留待 Sprint 6/7）：

- QC 场景 LIO 在 odom 系仍可能累积漂移（定位 RMSE 15–25 m）。
- INS yaw 与地图系 yaw 常值偏置需二次标定。
- GNSS/INS 仍未参与融合，仅作参考与通道记录。

---

## 4. 验收命令

```bash
cd ~/proj/lightning-lm_ROS1_ws && catkin_make
./devel/lib/lightning/test_relocalization_main_path   # 含 TestMapFrameAnchor

# 集成（60s bag, 0.5x, use_sim_time）
rosparam set use_sim_time true
rosbag record -O /tmp/eval_s5.bag /tf /localization/ins &
./devel/lib/lightning/run_loc_online \
  --config config/yangpu_qc.yaml \
  --init_x 238.74 --init_y -208.92 --init_z 0.07 \
  --init_qx 0.00806 --init_qy -0.00054 --init_qz 0.90942 --init_qw -0.41581 &
rosbag play --clock -r 0.5 -u 60 /path/to/qc.bag
# SIGINT 停止 record 与 loc

python3 scripts/eval_loc_accuracy.py /tmp/eval_s5.bag \
  --config config/yangpu_qc.yaml --align --window 60
```

---

## 5. 相关文档

- [Sprint 5 测试报告](../testing/sprint5_accuracy_test_report.md)
- [Sprint 4 开发记录](sprint4_init_and_lio_guard.md)
- [Sprint 3 测试报告](../testing/sprint3_accuracy_test_report.md)
