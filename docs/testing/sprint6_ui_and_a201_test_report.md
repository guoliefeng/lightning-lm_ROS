# Sprint 6 测试报告：UI 地图显示 + yangpu/A201 定位

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 数据：`/home/glf/dataDisk/hainan/yangpu/A201/2026-06-09-14-07-02_0.bag` + `_1.bag`（约 15s）  
> 配置：`config/yangpu_a201.yaml`  
> 地图：`/home/glf/dataDisk/hainan/yangpu/map/chunked`（73 chunks）

---

## 1. 测试环境

| 项 | 值 |
|----|-----|
| INS 初值 | `(308.40, -150.60, -0.10)`，四元数 `(-0.001, 0.003, 0.385, 0.923)` |
| `with_ui` | `true` |
| `map_frame` | `disabled`（A201 INS 与地图系接近，无需 Sprint 5 平移锚定） |
| 单测 | `test_relocalization_main_path`：**8 cases 通过** |

---

## 2. 问题复现与修复验证

### 2.1 修复前（用户反馈 / 首次 A201 跑）

| 检查项 | 结果 |
|--------|------|
| `loaded chunks: 73` | ✅ 地图索引加载成功 |
| `UI map refresh` | ❌ 无（UI 指针为 `nullptr`） |
| 外部 INS 初始化 | ❌ 精配准降分 → 回退 `chunk_67 (0,-500)` |
| UI 地图可见性 | ❌ 空白或显示错误区域 |

### 2.2 修复后（`/tmp/a201_loc2.log`）

| 检查项 | 结果 |
|--------|------|
| `runtime UI attached` | ✅ |
| `UI map refresh: 13 static chunks` | ✅ AttachUi 时（功能点原点邻域） |
| `UI map refresh: 15 static chunks` | ✅ SetInitialPose 后（**308,-150 邻域**） |
| 粗搜保留 | ✅ `keeping coarse result`（1.50 → 0.02 时保留粗搜） |
| 外部 INS 初始化 | ✅ `(308.786, -150.982, 0.149)`，`conf=1.50` |
| `init with external pose` | ✅ 未回退错误功能点 |
| `loc fps` | ✅ ~0.48 Hz 起步，后续 ~10 Hz |

---

## 3. 定位过程观察（A201）

### 3.1 正常项

- IMU 初始化完成（~16 帧）
- LIO 首帧 `3429 pts`，effect feat ~833
- GNSS / INS 里程计通道有数据（`state estimator received gnss/odometry`）
- NDT 外部初值在真实 INS 位置收敛

### 3.2 已知告警（非阻塞）

| 日志 | 说明 |
|------|------|
| `assign DR pose failed` | 首帧定位时 DR 时间轴未就绪，初始化后消失 |
| `LidarOdom定位时间戳回退` | bag `--clock` 与传感器 epoch 混用，Sprint 5 已用 ROS clock 缓解 TF |
| `Failed to find match for field 'ring'` | meta_cloud 缺 ring 字段，可忽略 |
| `XmlRpcClient Connection refused` | 启动瞬间 roscore 未就绪，重试后正常 |

### 3.3 未测项

- 长时（>60s）精度 RMSE（可用 `eval_loc_accuracy.py` + `yangpu_a201` 后续补测）
- 多 bag 连续播放（A201 共 131 段）

---

## 4. 结论

| 目标 | 状态 |
|------|------|
| 确认地图是否加载 | ✅ **已加载**（73 chunks）；问题在 UI 未接线 |
| UI 显示地图块 | ✅ 修复后 15 chunks 刷新至车辆位置 |
| A201 外部 INS 初始化 | ✅ 修复粗搜/精配准逻辑后成功 |
| 定位链路运行 | ✅ `loc fps`、LIO、NDT 均正常 |

---

## 5. 用户复现步骤（UI）

```bash
# 终端 1
roscore

# 终端 2
source ~/proj/lightning-lm_ROS1_ws/devel/setup.bash
rosparam set use_sim_time true
./devel/lib/lightning/run_loc_online \
  --config src/lightning-lm_ROS/config/yangpu_a201.yaml \
  --init_x 308.40 --init_y -150.60 --init_z -0.10 \
  --init_qx -0.001009 --init_qy 0.002793 \
  --init_qz 0.384632 --init_qw 0.923065

# 终端 3（等 loaded chunks 后）
rosbag play --clock -r 1.0 \
  /home/glf/dataDisk/hainan/yangpu/A201/2026-06-09-14-07-02_0.bag
```

窗口标题 **`UI`**：应看到灰色地图 + 绿色扫描轨迹 + 红色定位轨迹。

---

## 6. 相关文档

- [Sprint 6 开发记录](../development/sprint6_ui_map_display.md)
