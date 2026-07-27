# Sprint 5 测试报告：地图系锚定 + 精度复测

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 数据：`yangpu/qc/2026-06-22-12-08-26_0.bag`（前 60s，`--clock -r 0.5`）  
> 配置：`config/yangpu_qc.yaml`（含 `map_frame` 平移锚定）  
> 评估 bag：`/tmp/eval_s5c.bag`（TF 时间戳修复后录制）

---

## 1. 测试环境

| 项 | 值 |
|----|-----|
| 地图 | `/home/glf/dataDisk/hainan/yangpu/map/chunked`（73 chunks） |
| INS 初值（CLI，INS 系） | `(238.74, -208.92, 0.07)`，四元数 `(0.00806, -0.00054, 0.90942, -0.41581)` |
| 锚定后地图系初值 | `(0.233, -400.289, 0.955)` |
| NDT 初始化成功位姿 | `(0.632, -400.166, 1.378)` |
| 单测 | `test_relocalization_main_path`：**8 cases 全部通过**（含 `TestMapFrameAnchor`） |

---

## 2. 单元测试

```bash
./devel/lib/lightning/test_relocalization_main_path
# reloc main path smoke tests passed
```

`TestMapFrameAnchor` 覆盖：

- `mode: fixed`（平移 + yaw 90°）
- `mode: anchor_pair`（由两点求变换）

---

## 3. 集成测试步骤

```bash
rosparam set use_sim_time true
rosbag record -O /tmp/eval_s5c.bag /tf /localization/ins &
./devel/lib/lightning/run_loc_online \
  --config config/yangpu_qc.yaml \
  --init_x 238.74 --init_y -208.92 --init_z 0.07 \
  --init_qx 0.00806 --init_qy -0.00054 --init_qz 0.90942 --init_qw -0.41581 &
sleep 10
rosbag play --clock -r 0.5 -u 60 /home/glf/dataDisk/hainan/yangpu/qc/2026-06-22-12-08-26_0.bag
# SIGINT 停止 record / loc
```

**运行时观察**：

| 指标 | 结果 |
|------|------|
| `map_frame` 加载 | `T_ins_to_map t=[-238.507, -191.369, 0.885]` |
| NDT 初始化 | 成功，位姿与锚定初值一致（~400 m 负 Y 区域） |
| `loc fps` | 稳定后约 0.8–10 Hz（含初始化阶段） |
| LIO 退化拒绝 | 本次未大量触发；单帧 delta 多为厘米级 |
| TF 时间戳 | `map→odom` 与 `odom→base_link` 均为回放 clock（修复后） |

---

## 4. 精度评估结果

评估命令：

```bash
python3 scripts/eval_loc_accuracy.py /tmp/eval_s5c.bag \
  --config config/yangpu_qc.yaml --align --window 60
```

### 4.1 与 Sprint 3/4 对比（含 `--align`）

| Sprint | 平面 RMSE (m) | 说明 |
|--------|---------------|------|
| Sprint 3 | **587–799** | 无 `T_ins_to_map`，Umeyama 仍无法消除系间错位 |
| Sprint 4 | ~80+（同类问题） | 初始化/事件链改善，坐标系问题仍在 |
| **Sprint 5** | **17.6** | 应用 `T_ins_to_map` + TF 时间戳修复 |

### 4.2 Sprint 5 分项指标（`eval_s5c.bag`）

| 窗口 | 平面 mean | 平面 RMSE | 平面 p95 | 高程 RMSE | 航向 mean |
|------|-----------|-----------|----------|-----------|-----------|
| 前 60s + align | 16.3 m | **17.6 m** | 25.5 m | 0.58 m | 63.9° |
| 前 30s（静态偏多） | 24.2 m | 27.4 m | 43.6 m | 0.56 m | 57.0° |
| 前 30s + min-move 5m | 14.1 m | 15.5 m | 23.8 m | 0.58 m | 63.9° |

`--config` 输出：

```
map_frame anchor: trans [-238.507, -191.369, 0.885] m, yaw 0.00 deg
reference trajectory transformed by T_ins_to_map
est poses: 4019, ref poses: 3412
```

---

## 5. 结论

### 5.1 通过项

- [x] `MapFrameAnchor` 模块与 YAML 加载
- [x] `run_loc_online` INS 初值自动变换到地图系
- [x] `eval_loc_accuracy.py` 支持 `--config` 变换参考轨迹
- [x] TF 双链路时间戳统一，评估可稳定配对
- [x] 平面 RMSE 从 **数百米降至 ~18 m**（数量级改善）

### 5.2 未达验收目标

原 Sprint 5 建议验收：

| 目标 | 实际 | 原因 |
|------|------|------|
| 静态 30s 平面 RMSE < 2 m | **27.4 m** | 仅标定平移，**yaw 偏置 ~50–60° 未标定**；LIO/NDT 在 QC 场景仍有漂移 |
| 运动段 p95 < 5 m | **23.8 m** | 同上 + odom 系累积误差 |

### 5.3 后续建议（Sprint 6）

1. 用 `anchor_pair` 或 `--ins-to-map-yaw` 标定航向（预计可大幅压低 yaw 误差）。
2. 多点最小二乘标定 `T_ins_to_map`（首帧 + 运动段若干 INS/NDT 对应点）。
3. 继续 Phase A 闭环：PGO 平滑结果回写 TF / 限制 LIO 在 odom 系发散。

---

## 6. 相关文档

- [Sprint 5 开发记录](../development/sprint5_map_frame_anchor.md)
- [Sprint 4 测试报告](sprint4_accuracy_test_report.md)
