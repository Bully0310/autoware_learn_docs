# Autoware Diffusion Planner 算法详细分析

本文基于当前工作区 `autoware_diffusion_planner` 的 README、launch、参数、输入维度、预处理、TensorRT 推理和后处理代码，分析 Diffusion Planner 的主要组成、数学原理、工作流程、输入输出、关键参数及实际应用表现。

## 1. 核心定位

Autoware Diffusion Planner 是一个基于深度生成模型的 trajectory generator。它不采用传统的“规则模块逐条修改速度”方式，而是把 ego 历史、周围交通参与者、Lanelet2 地图、route、交通灯、限速、goal 和车辆形状共同编码为条件，直接生成未来多步的 ego trajectory，并可同时生成邻居对象的未来预测。

```text
历史状态 + 邻居 + 地图 + route + 交通灯 + goal
  -> 条件扩散模型 / TensorRT 推理
  -> 多步 pose 预测
  -> 速度计算、平滑、停止后处理
  -> CandidateTrajectories
  -> 新规划框架的 trajectory optimizer / selector
```

需要特别区分：

- **Diffusion Planner**：神经生成式轨迹规划器。
- **TensorRT 推理封装**：负责加载 ONNX/engine 和执行 GPU 推理。
- **后处理**：把 pose 预测转换为 Autoware `Trajectory`，计算速度、停车状态和 turn indicator。
- **Trajectory Optimizer**：在新 planning framework 中对候选轨迹进一步筛选/优化；它不是 Diffusion 模型本身。

## 2. 当前代码和运行边界

### 2.1 主要代码结构

```text
src/universe/autoware_universe/planning/autoware_diffusion_planner/
├── src/preprocessing/
│   ├── lane_segments.cpp       # Lanelet 几何/限速/信号特征
│   ├── traffic_signals.cpp     # 交通信号预处理
│   └── preprocessing_utils.cpp # 历史、归一化、随机轨迹
├── src/conversion/
│   ├── agent.cpp                # tracked object -> agent history
│   └── lanelet.cpp              # Lanelet2 -> tensor 特征
├── src/inference/
│   └── tensorrt_inference.cpp   # ONNX/TensorRT GPU 推理
├── src/postprocessing/
│   ├── postprocessing_utils.cpp # pose -> trajectory/predicted objects
│   └── turn_indicator_manager.cpp
├── src/diffusion_planner_node.cpp
├── config/diffusion_planner.param.yaml
└── launch/diffusion_planner.launch.xml
```

### 2.2 与当前 Autoware planning 的关系

README 给出的新 planning framework 集成方式是：

```text
/planning/mission_planning/route
        + localization/perception/map
          -> /planning/generator/diffusion_planner/candidate_trajectories
          -> autoware_trajectory_optimizer
          -> /planning/trajectory
```

在该集成配置中，Diffusion Planner 不再作为旧 `scenario_planning/velocity_smoother/trajectory` 的普通替代节点，而是作为 trajectory generator 产生候选轨迹；最终输出由 trajectory optimizer/selector 处理。Planning Validator 的旧输出会被 remap 到 `.../trajectory/unused`，说明这是一条迁移中的新规划链，不能和默认 lane-driving launch 混为一谈。

## 3. 主节点和运行周期

Diffusion Planner 是 ROS 2 node：

```text
autoware_diffusion_planner_node
```

默认参数：

```yaml
planning_frequency_hz: 10.0
batch_size: 1
```

节点启动时：

1. 加载 ONNX model、参数 JSON 和 TensorRT plugins。
2. 检查模型 major version 与代码期望版本。
3. 读取 normalization statistics。
4. 创建 TensorRT engine；engine 路径带 batch size 后缀。
5. 等待 vector map、route、odometry、acceleration、tracked objects 和 turn indicator 等数据。
6. 以 planning frequency 周期执行预处理、推理和发布。

`build_only=true` 时只构建/加载 TensorRT engine，完成后退出，适合部署前准备 engine。

## 4. 输入输出接口

### 4.1 输入

| 输入 | 默认类型 | 作用 |
| --- | --- | --- |
| `~/input/odometry` | `nav_msgs/msg/Odometry` | ego pose、速度、历史轨迹和当前状态 |
| `~/input/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | ego 当前加速度 |
| `~/input/tracked_objects` | `autoware_perception_msgs/msg/TrackedObjects` | 邻居历史和动态参与者状态 |
| `~/input/traffic_signals` | `TrafficLightGroupArray` | 交通灯状态及超时处理 |
| `~/input/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | Lanelet 几何、边界、道路类型和限速 |
| `~/input/route` | `autoware_planning_msgs/msg/LaneletRoute` | route lanelets 和目标 pose |
| `~/input/turn_indicators` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` | 历史转向灯状态 |

README 所列输入是模型输入构造的原始 ROS 数据；模型真正接收的是 ego frame 下归一化后的 tensor。

### 4.2 输出

| 输出 | 类型 | 含义 |
| --- | --- | --- |
| `~/output/trajectory` | `autoware_planning_msgs/msg/Trajectory` | batch 0 的主 ego trajectory |
| `~/output/trajectories` | `CandidateTrajectories` | batch 中所有候选轨迹和 generator info |
| `~/output/predicted_objects` | `autoware_perception_msgs/msg/PredictedObjects` | 可选的邻居未来预测 |
| `~/output/turn_indicators` | `TurnIndicatorsCommand` | 模型预测的转向灯指令 |
| `~/debug/lane_marker` | `MarkerArray` | Lane tensor 可视化 |
| `~/debug/route_marker` | `MarkerArray` | Route tensor 可视化 |
| `~/debug/processing_time_detail_ms` | processing time detail | 预处理/推理/后处理耗时 |

在 README 的新 planning framework 示例中，候选输出被 remap 为：

```text
/planning/generator/diffusion_planner/candidate_trajectories
```

## 5. 模型输入表示

### 5.1 Ego 历史

代码定义：

```text
INPUT_T = 30
EGO_HISTORY_SHAPE = {1, INPUT_T + 1, 4}
```

每个历史点主要表示：

```text
x, y, cos(yaw), sin(yaw)
```

历史会变换到当前 ego frame，并由 `ego_agent_past` 输入模型。使用 cos/sin 表示航向避免直接使用 yaw 在 $-\pi/\pi$ 边界处不连续。

### 5.2 当前 ego 状态

`ego_current_state` 的维度为 10。它由当前 odometry、acceleration 和车辆 wheelbase 等状态特征构造，具体字段顺序由模型 args JSON 和 preprocessing 实现共同定义。

### 5.3 邻居 agent

常量定义：

```text
MAX_NUM_NEIGHBORS = 32
NEIGHBOR_SHAPE = {1, MAX_NUM_NEIGHBORS, INPUT_T + 1, 11}
```

节点从 TrackedObjects 更新 agent histories：

1. 维护每个对象历史。
2. 变换到 ego-centric 坐标系。
3. 按距离/有效性选择最多 32 个邻居。
4. 对未知类别按 `ignore_unknown_neighbors` 过滤。
5. 通过 `ignore_neighbors` 可完全关闭邻居影响。

11 维特征的具体语义必须与模型 args/训练版本一致，不能只根据常量文件猜测。它通常包含位置、姿态、速度、尺寸/类别或有效性等 agent 状态。

### 5.4 Lane 和 route

模型同时使用一般 Lanelet context 和 route context：

```text
LANES:
  NUM_SEGMENTS_IN_LANE = 140
  POINTS_PER_SEGMENT = 20

ROUTE_LANES:
  NUM_SEGMENTS_IN_ROUTE = 25
  POINTS_PER_SEGMENT = 20
```

每个 lane segment 的特征包含：

```text
X, Y
dX, dY
left boundary X/Y
right boundary X/Y
traffic light encoding
lane type encoding
speed limit
```

Lane 和 route 会由 Lanelet2 地图转换到 ego frame，并带有 speed limit/traffic light 特征。一般 lane 提供局部道路上下文，route lane 提供任务相关拓扑约束。

### 5.5 Polygon、LineString 和目标

当前输入还包含：

```text
NUM_POLYGONS = 10
POINTS_PER_POLYGON = 40
NUM_LINE_STRINGS = 10
POINTS_PER_LINE_STRING = 20
goal_pose = [x, y, cos(yaw), sin(yaw)]
ego_shape = [wheel_base, vehicle_length, vehicle_width]
```

目标 pose 从 map frame 转换到 ego frame：

$$
T_{goal}^{ego}=(T_{ego}^{map})^{-1}T_{goal}^{map}
$$

模型只需学习相对位置和相对方向，因此不直接依赖全球坐标数值。

## 6. Diffusion 模型数学原理

### 6.1 条件轨迹生成

设场景条件为：

$$
c=\{H_{ego},H_{agents},M_{lane},M_{route},L_{traffic},G_{goal},S_{vehicle}\}
$$

其中 $H$ 是历史，$M$ 是地图/route，$L$ 是交通灯和限速，$G$ 是目标，$S$ 是车辆形状。

未来轨迹记为：

$$
\tau_0=\{(x_t,y_t,\cos\psi_t,\sin\psi_t)\}_{t=1}^{T}
$$

Diffusion Planner 学习条件分布：

$$
p_\theta(\tau_0\mid c)
$$

它与单一确定性回归不同，可以通过不同随机噪声样本产生多个候选未来。

### 6.2 前向扩散

经典 DDPM 将真实轨迹逐步加入高斯噪声：

$$
q(\tau_k\mid\tau_{k-1})=
\mathcal N\left(\sqrt{1-\beta_k}\tau_{k-1},\beta_kI\right)
$$

经过 $K$ 步后，轨迹接近标准高斯噪声：

$$
\tau_K\approx\mathcal N(0,I)
$$

Autoware 节点不执行训练阶段的 forward diffusion；它加载已经训练好的 ONNX 模型，在推理时提供 sampled trajectory/noise 和场景条件。

### 6.3 反向去噪

模型学习在条件 $c$ 下从噪声恢复轨迹的反向过程：

$$
p_\theta(\tau_{k-1}\mid\tau_k,c)
$$

抽象地表示为：

$$
\tau_{k-1}=f_\theta(\tau_k,c,k)+\sigma_k z_k
$$

其中 $f_\theta$ 是神经网络预测的去噪更新，$z_k$ 是随机噪声，$\sigma_k$ 是噪声调度项。重复执行到 $k=0$ 后得到未来轨迹。

在 Autoware 实现中，TensorRT engine 的网络输入包括 `sampled_trajectories` 和上述条件张量，输出包括：

```text
prediction: [batch, agents, output_time, pose_dim]
turn_indicator_logit
```

具体去噪步数、网络结构、beta schedule 和 loss 不在该 ROS 包中定义，而由 ONNX 模型和 `diffusion_planner.param.json` 决定。

### 6.4 多候选与温度

对于 batch 中第 $b$ 个随机样本：

$$
\tau^{(b)}\sim p_\theta(\tau\mid c;T_b)
$$

`temperature` 影响输入 sampled trajectories 的随机性/分布尺度。低温通常更接近确定性输出，高温可能产生更丰富但更不稳定的候选。节点把每个 batch 轨迹放入 `CandidateTrajectories`，batch 0 同时作为主 trajectory 发布。

## 7. 推理前处理流程

每个 planning cycle 的 `create_frame_context()` 包含：

```text
tracked objects -> agent history
odometry/acceleration -> ego state/history
traffic signals -> traffic_light_id_map
route/map -> lane segment context
turn indicator report -> indicator history
```

之后 `create_input_data()`：

1. 生成 sampled trajectories。
2. 生成 ego past/current state。
3. 生成 neighbor agents past。
4. 生成 lane/route lane tensor。
5. 生成 polygons/line strings。
6. 生成 goal pose 和 ego shape。
7. 复制 batch。
8. 依据 args JSON 的 normalization statistics 归一化。
9. 检查输入是否存在 invalid value。

## 8. TensorRT 推理和模型版本

### 8.1 GPU 推理

`TensorrtInference`：

- 加载 ONNX 文件。
- 加载自定义 TensorRT plugins。
- 使用 FP32 precision 配置。
- 按 batch size 创建 dynamic profile。
- 将各输入复制到 CUDA device。
- 执行 engine inference。
- 复制 prediction 和 turn indicator logit 到 host。

engine 文件命名包含 batch size，例如：

```text
diffusion_planner_batch1.engine
```

### 8.2 模型兼容性

代码常量：

```text
WEIGHT_MAJOR_VERSION = 3
```

README 说明：

- major version 改变表示模型输入/输出或架构改变，通常不兼容当前 ROS node。
- minor version 只更新权重时，在 major 兼容的情况下可以直接替换。

模型文件和 args JSON 必须来自同一版本；不能只替换 ONNX 而保留另一版本的 normalization/shape 配置。

## 9. 输出后处理

### 9.1 Pose 转 Trajectory

模型输出 pose 为：

```text
x, y, cos(yaw), sin(yaw)
```

节点将 ego batch 的 pose 从 ego frame 变换回 map frame：

$$
p_t^{map}=T_{ego}^{map}p_t^{ego}
$$

输出时间间隔固定为：

$$
\Delta t=0.1s
$$

`OUTPUT_T=80`，所以单个预测 horizon 约为 8 秒。

### 9.2 速度计算和平滑

相邻预测点的速度由距离差分计算：

$$
v_t=\frac{\|p_t-p_{t-1}\|}{\Delta t}
$$

节点使用 `velocity_smoothing_window` 平滑速度。若车辆正在移动且预测/距离结果低于 `stopping_threshold`，可以启用 force stop 逻辑，避免输出极小速度导致车辆无法稳定停住。

### 9.3 邻居预测

如果 `predict_neighbor_trajectory=true`，模型输出中的邻居部分会转换为 `PredictedObjects`：

- 预测时间步为 0.1 秒。
- 使用 batch 0。
- 保留对象原始分类、形状、ID 和当前运动状态。
- 生成 confidence 为 1.0 的 predicted path。

该输出可用于其他规划模块，但它是模型预测结果，不等于独立感知预测器的概率校准输出。

### 9.4 Turn Indicator

模型输出 `turn_indicator_logit`，由 `TurnIndicatorManager` 结合：

- 上一次车辆 turn indicator report。
- `turn_indicator_keep_offset`。
- `turn_indicator_hold_duration`。
- `KEEP` 输出类别。

得到最终 `TurnIndicatorsCommand`。因此模型预测的灯光状态还会经过时间保持/迟滞，而不是直接把 logit 映射成瞬时命令。

## 10. 关键参数

参数文件：

```text
src/universe/autoware_universe/planning/autoware_diffusion_planner/config/
  diffusion_planner.param.yaml
```

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `onnx_model_path` | v3.0 ONNX | 模型权重路径 |
| `args_path` | v3.0 JSON | 模型形状、权重版本和归一化配置 |
| `plugins_path` | TensorRT plugins `.so` | 自定义 TensorRT 层 |
| `planning_frequency_hz` | `10.0` | 推理周期 |
| `ignore_neighbors` | `false` | 是否完全忽略邻居对象 |
| `ignore_unknown_neighbors` | `true` | 是否忽略未知类别对象 |
| `predict_neighbor_trajectory` | `true` | 是否发布邻居预测 |
| `traffic_light_group_msg_timeout_seconds` | `0.2 s` | 交通灯消息有效期 |
| `batch_size` | `1` | 候选轨迹 batch 数 |
| `temperature` | `[0.0]` | 随机采样温度列表 |
| `velocity_smoothing_window` | `8` | 速度平滑窗口 |
| `stopping_threshold` | `0.3 m/s` | 小速度/force-stop 阈值 |
| `turn_indicator_keep_offset` | `-1.25` | KEEP 状态处理偏移 |
| `turn_indicator_hold_duration` | `1.0 s` | 灯光状态保持时间 |
| `shift_x` | `false` | 是否将模型参考中心沿车辆轴偏移 |
| `debug_params.publish_debug_route` | `true` | 是否发布 route marker |
| `debug_params.publish_debug_map` | `false` | 是否发布 lane marker |

### 10.1 参数调优建议

- 提高 `batch_size` 可增加候选多样性，但会增加显存和推理耗时；模型 engine profile 必须匹配。
- 增大 temperature 可能增强多样性，但应通过候选轨迹 optimizer/filter 约束风险。
- 增大 `velocity_smoothing_window` 可减少速度抖动，但会削弱对突然停车/短距离行为的响应。
- 降低 traffic signal timeout 会更快视为信号过期；在通信不稳定时可能造成交通灯条件缺失。
- `ignore_neighbors=true` 只适合隔离实验，不能用于正常动态交通场景。
- 修改 `stopping_threshold` 前应同时观察控制器的 keep-stopped 和速度跟踪逻辑。

## 11. 实际应用表现

### 11.1 对静态地图的响应

模型同时看到一般 lane context、route lane context、边界 polygon 和 line string，因此能够在 lane-level 地图条件下生成符合道路形状的轨迹。地图 lane type、交通灯和限速特征质量会直接影响生成结果。

### 11.2 对邻居车辆/行人的响应

当 `ignore_neighbors=false` 时，模型利用邻居历史和当前状态生成条件轨迹。典型表现是：

- 前车减速时 ego 轨迹提前减速。
- 邻居进入 ego path 时轨迹产生横向或纵向响应。
- 多候选 batch 可能产生不同的避让/跟车策略。

如果对象 history 不连续、frame 错误或 prediction 输入为空，模型可能退化为只依赖地图和 ego 历史的输出。

### 11.3 交通灯和限速

lane segment tensor 中包含 traffic light encoding 和 speed limit；traffic signal 经过 timeout 后写入内部交通灯 map。信号过期或 route lane 匹配失败时，模型可能表现为速度策略不稳定，因此应检查：

```text
traffic signal timestamp
route lane ids
lane speed limit tensor
valid lane/route counts
```

### 11.4 轨迹平滑和停车

模型生成的是 pose 序列，速度由后处理差分计算。短时间内相邻 pose 距离变化会放大速度噪声，因此 `velocity_smoothing_window` 对控制体验很重要。模型本身输出低速点并不保证车辆稳定停止，force stop 和下游 trajectory optimizer/validator 仍然重要。

### 11.5 计算和部署表现

该模块依赖 CUDA、TensorRT plugins、ONNX 模型和正确的 engine build。实际瓶颈包括：

- TensorRT engine 首次构建耗时。
- batch size 和候选数量。
- 预处理 Lanelet/agent tensor 构造。
- GPU 显存和 host-device copy。
- 10 Hz 推理周期内的模型和后处理耗时。

节点会发布 `inference_status` diagnostic 和 `processing_time_detail_ms`，应将其作为部署验收依据。

## 12. 可操作的验证流程

### 12.1 模型文件检查

```bash
ls -lh "$HOME/autoware_data/diffusion_planner/v3.0/"
ros2 param get /trajectory_generator/diffusion_planner_node onnx_model_path
ros2 param get /trajectory_generator/diffusion_planner_node args_path
```

确认 ONNX、JSON、plugins `.so` 存在且 major version 匹配。

### 12.2 节点和输出检查

```bash
ros2 node list | grep diffusion
ros2 topic info /planning/generator/diffusion_planner/candidate_trajectories -v
ros2 topic echo /planning/generator/diffusion_planner/candidate_trajectories --once
ros2 topic echo /planning/trajectory --once
ros2 topic echo /planning/turn_indicators_cmd --once
ros2 topic echo /diagnostics --once
```

### 12.3 输入完整性检查

```bash
ros2 topic hz /localization/kinematic_state
ros2 topic hz /localization/acceleration
ros2 topic hz /perception/object_recognition/tracking/objects
ros2 topic echo /planning/mission_planning/route --once
ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once
```

节点在缺少 objects、odometry、acceleration、route 或 turn indicator 时会跳过当前 inference；map 未加载时也不会生成轨迹。

### 12.4 三组推荐实验

#### 实验 A：无邻居地图跟踪

设置 `ignore_neighbors=true`，固定 route 和 map，观察模型是否生成平滑 lane-following trajectory。该实验用于隔离模型地图/ego 条件。

#### 实验 B：动态车辆响应

逐步放置前车、侧向切入车辆和停止车辆，比较：

- candidate trajectory 数量和差异。
- batch 0 trajectory。
- predicted_objects。
- 下游 trajectory optimizer 选择结果。

#### 实验 C：交通灯与 turn indicator

改变交通灯状态、限速和 route turn direction，记录：

```text
traffic signal timeout
route lane match
candidate speed profile
turn_indicator_logit/command
turn_indicator hold duration
```

## 13. 常见问题定位

| 现象 | 首先检查 | 可能原因 |
| --- | --- | --- |
| 节点不输出 trajectory | model/map/input diagnostics | ONNX/TensorRT 未加载、map 或必需输入缺失 |
| TensorRT engine 构建失败 | ONNX、plugins、batch size、CUDA | 模型版本或插件不匹配 |
| 输出轨迹为空/跳帧 | odometry、acceleration、route、turn report | polling subscriber 缺数据 |
| 轨迹不遵守 route | route tensor、lane valid count、goal pose | route lane 选取或 map frame 错误 |
| 动态目标无响应 | tracked objects history、过滤参数 | `ignore_neighbors`、unknown filter、对象 ID/history 问题 |
| 交通灯响应异常 | signal timeout、lane signal mapping | 信号过期或 route lane 不匹配 |
| 速度抖动 | velocity smoothing window、模型 pose 间距 | 后处理差分噪声或模型输出抖动 |
| 车辆不稳定停车 | stopping threshold、force stop、下游 optimizer | 小速度未归零或控制器停止策略冲突 |
| 候选很多但最终行为单一 | trajectory optimizer/selector | 下游筛选代价或安全约束淘汰候选 |
| 推理频率不足 | processing time、batch size、GPU | engine、预处理或候选数量过重 |

## 14. 安全和验证边界

Diffusion Planner 是生成式模型，输出不能仅凭模型推理成功就视为安全。生产集成至少需要：

1. 输入有效性和时间戳诊断。
2. 轨迹有限值、曲率、速度和加速度检查。
3. 车辆 footprint 与障碍物/道路边界碰撞检查。
4. candidate trajectory 的安全筛选/优化。
5. 轨迹超时和模型 inference failure 的 fallback。
6. GPU/TensorRT 资源和模型版本验收。

模型理论上学习的是条件分布：

$$
p_\theta(\tau\mid c)
$$

而不是显式证明：

$$
\forall t,\quad F(\tau_t)\subseteq D_t
$$

因此，地图可行域、碰撞和动力学验证仍必须由下游确定性模块承担。

## 15. 与传统规划算法的比较

| 方面 | Diffusion Planner | 规则/优化规划 |
| --- | --- | --- |
| 轨迹生成 | 条件生成模型直接生成多步 pose | 规则、采样或 QP/MPT 逐步求解 |
| 多模态行为 | batch/temperature 可产生多个候选 | 需要显式枚举候选或行为模块 |
| 地图使用 | 学习 lane/route/traffic 特征 | 显式几何和约束 |
| 可解释性 | 依赖输入 tensor、模型版本和 debug | 参数/规则/约束较直接 |
| 计算依赖 | GPU、TensorRT、模型 engine | CPU/GPU 取决于优化器 |
| 安全保证 | 需要下游验证 | 约束可显式表达，但也可能局部失败 |
| 迁移风险 | 数据分布和模型版本敏感 | 地图/参数/模型误差敏感 |

## 16. 总结

Autoware Diffusion Planner 的完整链路是：

```text
odometry/acceleration/objects/map/route/traffic signals
  -> 历史和 Lanelet 条件编码
  -> ego-centric normalization
  -> sampled trajectory + 条件扩散模型
  -> TensorRT GPU inference
  -> 80 步 pose prediction
  -> map-frame trajectory、速度平滑、force stop
  -> candidate trajectories / predicted objects / turn indicators
  -> trajectory optimizer 和安全验证
```

它的优势是能在统一模型中融合地图、交通参与者、信号和目标，并通过 batch/temperature 表达多种未来；它的工程关键则是模型版本、输入 tensor 一致性、GPU 推理实时性和下游安全过滤。排查问题时应沿“模型文件 -> 输入完整性 -> tensor valid count -> inference diagnostic -> postprocessing -> candidate selector -> final trajectory”的顺序进行。 