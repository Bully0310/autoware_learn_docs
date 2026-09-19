# Autoware Scenario Planning 算法详细分析

本文基于当前工作区的 `tier4_planning_launch`、Universe Planning 源码、默认 planning preset 和相关消息接口，分析 Autoware Scenario Planning 的组成、数学原理、运行流程、参数、输入输出和实际表现。

## 1. 核心结论

Scenario Planning 不是一个单独的规划器，而是一个根据当前道路场景，把 Mission Planning 的路线转换为可执行 path/trajectory 的分层系统：

```text
Mission route
  -> Scenario Selector
  -> Lane Driving 或 Parking
       Lane Driving:
         Behavior Path Planning
         -> Behavior Velocity Planning
         -> Motion Path Smoothing
         -> Kinematic Path Optimization/Sampling
         -> Motion Velocity Planning
       Parking:
         Costmap Generation
         -> Freespace Planning
  -> Scenario Selector 选择当前场景轨迹
  -> Velocity Smoother
  -> Planning Validator
```

它解决的是三个层次的问题：

1. **场景层**：当前车辆应执行 lane driving 还是 parking。
2. **行为层**：保持车道、换道、绕障、起步、靠边、路口通行等。
3. **运动层**：让路径满足车辆几何/运动学约束，并给轨迹施加障碍物、边界、加速度和 jerk 约束。

## 2. 代码入口和默认配置

### 2.1 主要 launch

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/
├── scenario_planning/scenario_planning.launch.xml
├── scenario_planning/lane_driving.launch.xml
├── scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
├── scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml
└── scenario_planning/parking.launch.xml
```

顶层 `scenario_planning.launch.xml` 做四件事：

- 启动 `autoware_scenario_selector`。
- 启动外部速度限制选择器和 Autoware Core 的 `autoware_velocity_smoother`。
- 启动 lane driving 分支和 parking 分支。
- 启动 hazard lights selector。

### 2.2 默认 preset 的主要选择

配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/planning/preset/default_preset.yaml
```

当前默认值体现的主要算法链为：

| 层级 | 默认选择 |
| --- | --- |
| Behavior Path | `behavior_path_planner` |
| 静态障碍物绕行 | 开启 |
| 左右换道 | 开启 |
| 动态障碍物绕行 | 关闭 |
| Sampling Planner | 关闭，实验功能 |
| Motion Path Smoother | `elastic_band` |
| Motion Path Planner | `path_optimizer` |
| Motion Velocity | obstacle stop/slowdown/cruise、dynamic stop、out-of-lane、velocity limiter、run-out、boundary prevention、road-user stop 均开启 |
| Velocity Smoother | `JerkFiltered` |
| Parking | 开启 |
| Validator | latency、trajectory、intersection collision、rear collision 均开启 |

因此，源码中存在的模块不一定属于当前实际运行链路。分析运行行为时，应同时看 preset、launch 条件和 `launch_modules` 参数。

## 3. 总体数据流和接口

```mermaid
flowchart TD
    ROUTE[/planning/mission_planning/route\nLaneletRoute]
    MAP[/map/vector_map\nLaneletMapBin]
    ODOM[/localization/kinematic_state\nOdometry]
    ACC[/localization/acceleration]
    OBJ[/perception/object_recognition/objects]
    PCL[/perception/obstacle_segmentation/pointcloud]
    TL[/perception/traffic_light_recognition/traffic_signals]
    GRID[/perception/occupancy_grid_map/map]

    ROUTE --> SELECT[Scenario Selector]
    ODOM --> SELECT
    SELECT -->|lane driving| BP[Behavior Path Planner]
    SELECT -->|parking| CG[Costmap Generator]
    CG --> FS[Freespace Planner]
    MAP --> CG
    OBJ --> CG
    PCL --> CG
    CG --> FS
    ROUTE --> FS
    ODOM --> FS
    FS --> PARK[Parking trajectory]

    ROUTE --> BP
    MAP --> BP
    ODOM --> BP
    ACC --> BP
    OBJ --> BP
    GRID --> BP
    BP --> PATH[PathWithLaneId]
    PATH --> BV[Behavior Velocity Planner]
    MAP --> BV
    OBJ --> BV
    TL --> BV
    BV --> BPATH[行为速度约束后的 path]

    BPATH --> SM[Elastic Band Smoother]
    SM --> PO[Path Optimizer]
    PO --> MVP[Motion Velocity Planner]
    MAP --> MVP
    ODOM --> MVP
    ACC --> MVP
    OBJ --> MVP
    PCL --> MVP
    TL --> MVP
    GRID --> MVP
    MVP --> LANE[Lane driving trajectory]

    LANE --> SELECT
    PARK --> SELECT
    SELECT --> SCENARIO_TRAJ[/planning/scenario_planning/scenario_selector/trajectory]
    SCENARIO_TRAJ --> VS[Velocity Smoother]
    VS --> FINAL[/planning/scenario_planning/velocity_smoother/trajectory]
    FINAL --> VALIDATOR[Planning Validator]
    VALIDATOR --> OUTPUT[/planning/trajectory]
```

### 3.1 主要输入

| 输入 | 用途 |
| --- | --- |
| `/planning/mission_planning/route` | 起点、目标点和 lanelet route sections |
| `/map/vector_map` | lanelet、停止线、路口、横道、停车区域等语义地图 |
| `/localization/kinematic_state` | ego pose、速度和姿态 |
| `/localization/acceleration` | 速度平滑和纵向约束 |
| `/perception/object_recognition/objects` | 动态/静态道路参与者 |
| `/perception/obstacle_segmentation/pointcloud` | 无地面点云和障碍物几何 |
| `/perception/traffic_light_recognition/traffic_signals` | 交通灯状态 |
| `/perception/occupancy_grid_map/map` | 占据栅格障碍物表示 |
| `/system/operation_mode/state` | 自动/手动/外部模式和状态切换 |

### 3.2 主要输出

| 输出 | 生产阶段 | 含义 |
| --- | --- | --- |
| `.../behavior_planning/path_with_lane_id` | Behavior Path | 带 lane id 的行为路径 |
| `.../behavior_planning/path` | Behavior Velocity | 叠加交通规则速度约束的 path |
| `.../motion_planning/path_smoother/path` | Path Smoother | 平滑后的 path |
| `.../motion_planning/path_optimizer/trajectory` | Optimizer/Sampler | 运动学可行的轨迹 |
| `/planning/scenario_planning/lane_driving/trajectory` | Motion Velocity | lane driving 最终候选轨迹 |
| `/planning/scenario_planning/parking/trajectory` | Freespace Planner | parking 候选轨迹 |
| `/planning/scenario_planning/scenario_selector/trajectory` | Scenario Selector | 当前场景选择后的轨迹 |
| `/planning/scenario_planning/velocity_smoother/trajectory` | Velocity Smoother | 加速度/jerk 平滑后的轨迹 |
| `/planning/trajectory` | Validator | 交给 Control 的最终轨迹 |

## 4. Scenario Selector：场景仲裁

代码包：

```text
src/universe/autoware_universe/planning/autoware_scenario_selector/
```

其输入包括 lane driving trajectory、parking trajectory、route、vector map、odometry、operation mode 和 parking completed 状态；输出当前 `Scenario` 与被选择的 `Trajectory`。

### 4.1 状态转换

典型转换为：

```text
未初始化
  -> 当前 pose 在 lane 内       -> LaneDriving
  -> 当前 pose 不在 lane 内     -> Parking

LaneDriving
  -> goal 在 parking lot 且不在 lane 内 -> Parking

Parking
  -> parking completed 且车辆回到 lane -> LaneDriving
```

Selector 并不重新生成轨迹，而是在两个候选生产分支中选择当前可跟踪的轨迹。为了避免切换时输出空轨迹，它会检查场景是否完成、输入是否 ready 以及 trajectory 是否为空。

### 4.2 数学抽象

令当前车辆位置为 $p_e$，道路可行驶区域为 $\mathcal L$，停车区域为 $\mathcal P$。可以把场景判定抽象为：

$$
\operatorname{inLane}(p_e)=
\begin{cases}
1,&p_e\in\mathcal L\\
0,&p_e\notin\mathcal L
\end{cases}
$$

初始化时：

$$
s_0=
\begin{cases}
\text{LaneDriving},&\operatorname{inLane}(p_e)=1\\
\text{Parking},&\operatorname{inLane}(p_e)=0
\end{cases}
$$

LaneDriving 切换到 Parking 的条件可表示为：

$$
s=\text{LaneDriving}\land p_g\in\mathcal P\land p_g\notin\mathcal L
$$

Parking 回到 LaneDriving 需要同时满足停车已完成和车辆重新位于 lane：

$$
s=\text{Parking}\land C_{parking}=1\land p_e\in\mathcal L
$$

这里的公式是对源码状态逻辑的形式化表达；实际 `in lane` 和 parking completed 的判断仍由 Lanelet2 查询与 Freespace Planner 状态提供。

## 5. Lane Driving：Behavior Path Planning

代码入口：

```text
src/universe/autoware_universe/planning/behavior_path_planner/
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/
  scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
```

### 5.1 框架工作原理

`BehaviorPathPlannerNode` 是一个 Scene Module Manager 框架，而不是每个行为一个独立 ROS node。模块按 launch 参数组成字符串并动态加载：

```text
route + map + odometry + objects + occupancy grid
  -> module manager 更新每个 scene module
  -> 判断模块是否 ready / running / success / failure
  -> 生成候选 path
  -> 处理模块优先级、互斥和 RTC approval
  -> 输出 PathWithLaneId
```

当前 launch 支持的主要 path 模块：

- Static Obstacle Avoidance：静态障碍物横向绕行。
- Dynamic Obstacle Avoidance：动态目标避让。
- Lane Change Left/Right：左右换道。
- Avoidance by Lane Change：用换道替代道路内绕障。
- Sampling Planner：候选路径采样。
- Goal/Start Planner：目标段和起步段特殊处理。
- Side Shift：侧向平移。
- Bidirectional Traffic：双向交通和临时借道。

### 5.2 候选路径的数学目标

对参考路径 $r(s)$，候选路径可用 Frenet 形式表示为：

$$
p(s)=r(s)+d(s)n(s)
$$

其中 $s$ 是沿参考路径的弧长，$d(s)$ 是横向偏移，$n(s)$ 是参考路径法向量。行为路径模块的实际约束可抽象为：

$$
\min_{d(s)} J[d]
$$

$$
J=\int_0^{S}\left(
w_d d(s)^2+w_{d'}d'(s)^2+w_{d''}d''(s)^2+w_{obs}C_{obs}(s)
\right)ds
$$

同时满足：

$$
d_{min}(s)\leq d(s)\leq d_{max}(s)
$$

$$
\operatorname{Footprint}(p(s))\cap\mathcal O(s)=\varnothing
$$

以及车道边界、目标 lanelet、换道间隙和曲率变化约束。不同 scene module 并不一定直接求解同一个连续优化问题；上述目标函数是对“候选生成 + 碰撞/边界筛选 + 舒适性代价排序”的统一数学抽象。

### 5.3 换道安全间隙

换道模块需要判断目标车道上的前后目标。对目标车辆与 ego 的纵向相对距离 $\Delta s$、相对速度 $\Delta v$，常用时间间隙抽象为：

$$
T_{gap}=\frac{\Delta s}{\max(v_{rel},\epsilon)}
$$

其中 $v_{rel}$ 根据目标在前方或后方采用相应的 closing speed，$\epsilon$ 避免除零。只有前后间隙满足配置的安全距离/时间阈值、预测冲突不发生且换道几何可行时，模块才可能从准备状态进入执行状态。

实际模块还会结合：

- 目标车道是否与 route sections 相容。
- 车辆 footprint 是否跨越车道边界。
- 换道路径的横向偏移和曲率。
- RTC approval 是否允许执行。
- 当前行为模块是否与避障模块冲突。

### 5.4 静态障碍物绕行

静态障碍物绕行的基本几何条件是车辆 footprint 与障碍物膨胀区域不相交。若障碍物集合为 $\mathcal O$，车辆 footprint 为 $F$，安全膨胀半径为 $r_{safe}$，则：

$$
\mathcal O^+=\mathcal O\oplus B(r_{safe})
$$

候选路径 $p(s)$ 需要满足：

$$
F(p(s))\cap\mathcal O^+=\varnothing,\quad s\in[0,S]
$$

如果道路剩余宽度不足以产生安全横向偏移，行为路径规划不能强行绕障，后续 Motion Velocity Planner 可能选择减速或停车。

## 6. Behavior Velocity Planning：规则速度规划

Behavior Velocity Planner 接收行为路径，在不改变主要横向意图的情况下生成停止点、速度上限和减速区。模块通过 `launch_modules` plugin 列表装载，常见模块包括：

- Traffic Light、Stop Line。
- Crosswalk、Walkway。
- Intersection、Roundabout。
- Blind Spot、Detection Area、Occlusion Spot。
- Virtual Traffic Light。
- Speed Bump。
- No Stopping Area、No Drivable Lane。

### 6.1 停止点计算

若停止线沿 path 的弧长坐标为 $s_{stop}$，车辆当前弧长位置为 $s_0$，则停止距离为：

$$
d_{stop}=s_{stop}-s_0
$$

在理想恒定减速度模型下，使车辆在停止线前停止所需的减速度为：

$$
a_{req}=-\frac{v_0^2}{2d_{stop}}
$$

如果 $|a_{req}|$ 超过允许减速度，系统应更早开始减速，或由速度平滑器生成满足约束的速度曲线。实际 Behavior Velocity 模块通常先设置停止点/速度约束，最终速度连续性由 Motion Velocity 和 Velocity Smoother 共同处理。

### 6.2 速度上限合并

设不同规则插件在轨迹位置 $s$ 上提供速度上限 $v_i(s)$，外部限速为 $v_{ext}(s)$，道路默认限速为 $v_{map}(s)$，则统一上限可抽象为：

$$
v_{max}(s)=\min\left(v_{map}(s),v_{ext}(s),v_1(s),\ldots,v_n(s)\right)
$$

停车约束可以视为 $v_i(s_{stop})=0$。这解释了多个插件同时激活时车辆表现会趋于保守：任何有效的更低速度上限都可能成为最终约束。

### 6.3 交通灯决策

交通灯模块根据 stop line、车辆距离、当前速度和信号状态决定是否应停车。对红灯或不可通行状态，通常要求：

$$
v(s_{stop})=0
$$

对绿灯，停止约束可以被清除；对黄灯或信号过期，则由模块参数和安全策略决定是停车还是继续通过。当前 launch 中交通灯信息的超时参数为 `traffic_light_signal_timeout`，过期信号不应被无限期当作有效绿灯/红灯。

## 7. Motion Planning：几何平滑和车辆可行性

### 7.1 Path Smoother

默认 `motion_path_smoother_type=elastic_band`。它把离散 path 看作受约束的弹性带，通过平滑相邻点减少曲率和横向抖动。其一般数学形式可以写成：

$$
\min_{p_0,\ldots,p_N}
\sum_{i=1}^{N-1}
\left(
\lambda_s\|p_{i+1}-2p_i+p_{i-1}\|^2
 +\lambda_r\|p_i-r_i\|^2
\right)
$$

其中第一项抑制离散二阶差分，第二项避免偏离行为路径 $r_i$ 过多。实际 Elastic Band 实现还会结合车辆参数、odometry、边界和路径采样间隔。

### 7.2 Path Optimizer

默认 `motion_path_planner_type=path_optimizer`。其目标是把平滑路径调整为车辆运动学更容易跟踪的轨迹。常见约束包括：

$$
\kappa(s)\leq\kappa_{max}(v),
\qquad
|\Delta\kappa/\Delta s|\leq K_{max}
$$

其中 $\kappa$ 是曲率，$\kappa_{max}$ 与车辆转向能力、速度和参数有关。优化通常还需要满足道路边界和原始行为路径的偏离限制。

### 7.3 Path Sampler

如果 preset 将 planner 类型改为 `path_sampler`，系统会对不同横向偏移、曲率或 Frenet 参数采样候选，并使用碰撞、道路边界、曲率和舒适性代价选择：

$$
p^*=\arg\min_{p\in\mathcal P_{valid}}J(p)
$$

其中 $\mathcal P_{valid}$ 只包含通过碰撞和运动学检查的候选。当前默认 preset 不启用该实验模块。

## 8. Motion Velocity Planning：轨迹级速度约束

代码入口：

```text
src/universe/autoware_universe/planning/motion_velocity_planner/
```

`MotionVelocityPlannerNode` 接收 `path_optimizer/trajectory`，并通过 module plugin 修改最终 trajectory 的速度、加速度或限速候选。当前 launch 组装：

- `ObstacleStopModule`：障碍物前停车。
- `ObstacleSlowDownModule`：障碍物附近减速。
- `ObstacleCruiseModule`：障碍物条件下安全巡航。
- `DynamicObstacleStopModule`：动态目标冲突停车。
- `OutOfLaneModule`：轨迹越出车道时降速。
- `ObstacleVelocityLimiterModule`：生成障碍物速度上限。
- `RunOutModule`：处理突然进入路径的道路参与者。
- `BoundaryDeparturePreventionModule`：防止边界驶离。
- `RoadUserStopModule`：对道路参与者施加停车约束。

### 8.1 轨迹上的安全距离

对沿轨迹弧长方向距离 $d$ 的静态障碍物，在简化制动模型下，最低安全距离可写为：

$$
d_{safe}=d_{reaction}+d_{brake}+d_{margin}
$$

$$
d_{reaction}=v\,T_r
$$

$$
d_{brake}=\frac{v^2}{2a_{comfort}}
$$

其中 $T_r$ 是反应/系统延迟，$a_{comfort}$ 是允许的舒适减速度，$d_{margin}$ 是车辆尺寸和感知不确定性的余量。若障碍物距离满足 $d<d_{safe}$，Obstacle Stop 需要生成停车约束；若距离更大但风险存在，Obstacle Slow Down 可能提供较低 $v_{max}$。

### 8.2 动态障碍物冲突

设 ego 轨迹位置为 $p_e(t)$，动态目标预测位置为 $p_o(t)$，两者安全距离为 $d_{safe}(t)$，则冲突条件可抽象为：

$$
\exists t\in[t_0,t_0+T]:
\|p_e(t)-p_o(t)\|<d_{safe}(t)
$$

如果存在冲突，Motion Velocity Planner 可以通过降低速度、插入停止约束或发布 stop reason 处理。它通常不改变任务路线；横向绕行仍应回到 Behavior Path Planner。

### 8.3 边界驶离

对轨迹 footprint $F(s)$ 和可行驶区域 $\mathcal D$，边界安全条件是：

$$
F(s)\subseteq\mathcal D,\quad s\in[0,S]
$$

当轨迹接近边界时，Boundary Departure Prevention 可能通过速度限制给下游更多反应时间；如果几何上已经不可恢复，则应减速或停车，而不是仅靠速度模块把越界路径变成安全路径。

## 9. Parking/Freespace Planning

停车分支位于：

```text
scenario_planning/parking.launch.xml
```

### 9.1 Costmap Generator

`autoware_costmap_generator` 的输入为 objects、无地面点云、vector map 和当前 scenario，输出 grid map 与 occupancy grid。它把道路、障碍物和停车区域统一到栅格代价空间：

$$
C(x,y)=C_{map}(x,y)+C_{obs}(x,y)+C_{margin}(x,y)
$$

障碍物通常需要按车辆 footprint 和安全余量膨胀：

$$
\mathcal O^+=\mathcal O\oplus F\oplus B(r_{safe})
$$

因此 freespace planner 检查的不是传感器点是否直接落在路径上，而是车辆占用区域是否进入高代价/不可行驶栅格。

### 9.2 Freespace Planner

`autoware_freespace_planner` 接收 Mission route、scenario、occupancy grid 和 odometry，输出：

```text
/planning/scenario_planning/parking/trajectory
/planning/scenario_planning/parking/is_completed
```

它在二维自由空间中搜索满足车辆运动学的停车路径。典型搜索可抽象为状态空间图：

$$
x=(x,y,\psi),\qquad u=(v,\delta)
$$

车辆运动学自行车模型为：

$$
\dot x=v\cos\psi,
\qquad
\dot y=v\sin\psi,
\qquad
\dot\psi=\frac{v}{L}\tan\delta
$$

候选状态必须满足：

$$
F(x)\cap\mathcal O^+=\varnothing
$$

并且满足转向角、最小转弯半径、速度和地图边界约束。搜索目标通常是到达 route/goal 指定的停车区域，并最小化路径长度、倒车次数、转向变化和碰撞风险等代价：

$$
J(\pi)=w_lL(\pi)+w_rN_{reverse}(\pi)+w_\delta J_{steering}(\pi)+w_cJ_{collision}(\pi)
$$

具体搜索算法和代价由 `autoware_freespace_planning_algorithms` 与配置决定；上式是用于理解优化方向的统一抽象。

## 10. Velocity Smoother：全场景速度连续性

Scenario Selector 输出当前场景 trajectory 后，统一进入 Autoware Core 的 `autoware_velocity_smoother`：

```text
/planning/scenario_planning/scenario_selector/trajectory
  -> /planning/scenario_planning/velocity_smoother/trajectory
```

当前默认 `velocity_smoother_type=JerkFiltered`。速度规划可抽象为在离散轨迹点 $s_i$ 上求速度 $v_i$：

$$
\min_{v_i,a_i,j_i}
\sum_i \left(
w_v(v_i-v_i^{ref})^2+w_a a_i^2+w_j j_i^2
\right)
$$

满足：

$$
0\leq v_i\leq v_{max}(s_i)
$$

$$
a_{min}\leq a_i\leq a_{max}
$$

$$
j_{min}\leq j_i\leq j_{max}
$$

其中离散近似为：

$$
a_i\approx\frac{v_{i+1}^2-v_i^2}{2\Delta s_i},
\qquad
j_i\approx\frac{a_{i+1}-a_i}{\Delta t_i}
$$

它解决的是“上游插件给出的停止点和速度上限如何变成平滑、可跟踪的速度曲线”，不是重新选择 lane 或行为。

## 11. 关键参数

### 11.1 Behavior Path Planner

当前 `behavior_path_planner.param.yaml` 中值得重点关注的参数：

| 参数 | 默认值 | 影响 |
| --- | ---: | --- |
| `planning_hz` | `10.0` | 行为路径更新频率 |
| `backward_path_length` | `5.0 m` | 输出路径向车辆后方延伸长度 |
| `forward_path_length` | `300.0 m` | 输出路径向前预测长度 |
| `minimum_pull_over_length` | `16.0 m` | 靠边停车行为的最小有效长度 |
| `refine_goal_search_radius_range` | `7.5 m` | 目标附近 lanelet 搜索/修正范围 |
| `input_path_interval` | `2.0 m` | 输入 path 采样间隔 |
| `output_path_interval` | `2.0 m` | 输出 path 采样间隔 |
| `traffic_light_signal_timeout` | `1.0 s` | 交通灯信号过期时间 |
| `enable_akima_spline_first` | `false` | 是否优先进行 Akima spline 插值 |
| `enable_cog_on_centerline` | `false` | 是否将车辆质心相关处理放到中心线上 |

模块开关在 `default_preset.yaml` 中，而 scene module 的距离、间隙、避障和舒适性参数主要在 `autoware_launch/config/planning/scenario_planning/...` 下。

### 11.2 Behavior Velocity Planner

常用模块参数包括：

- traffic light：信号超时、停止线前余量、黄灯决策。
- crosswalk：行人识别范围、减速距离、停止距离。
- intersection：路口进入速度、冲突区域和遮挡策略。
- blind spot/detection area：检测区域、等待和停止条件。
- stop line：停止线识别和停车位置偏移。
- no stopping area：是否允许在特殊区域产生停车点。

这些参数的真实键名应以当前 `autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/` 下 YAML 为准。

### 11.3 Motion Planning

| 参数/选择 | 默认值或入口 | 影响 |
| --- | --- | --- |
| `motion_path_smoother_type` | `elastic_band` | 是否对行为路径做弹性带平滑 |
| `motion_path_planner_type` | `path_optimizer` | 使用 Path Optimizer、Path Sampler 或直接转换 |
| Elastic Band 参数 | `elastic_band_smoother.param.yaml` | 平滑强度、车辆约束和路径偏离 |
| Path Optimizer 参数 | `path_optimizer.param.yaml` | 曲率、边界和运动学可行性 |
| Path Sampler 参数 | `path_sampler.param.yaml` | 候选数量、采样范围和代价 |

### 11.4 Motion Velocity 和全局速度

Motion Velocity 模块主要参数在：

```text
.../motion_planning/motion_velocity_planner/
```

常见调参对象是障碍物停止距离、减速距离、动态目标预测范围、边界余量、道路参与者类别和速度上限。全局速度平滑参数在：

```text
.../scenario_planning/common/autoware_velocity_smoother/
```

应同时记录最大速度、最大减速度和 jerk，否则只修改障碍物阈值很难解释最终停车距离。

## 12. 实际运行表现

### 12.1 正常 lane driving

正常链路的可观察结果：

```text
route
  -> path_with_lane_id
  -> lane_driving/trajectory
  -> scenario_selector/trajectory
  -> velocity_smoother/trajectory
```

车辆通常沿 preferred lane 行驶；遇到变道或静态障碍物时，path 的横向形状和 lane id 发生变化；遇到红灯、横道或障碍物时，path 可能不变但速度曲线下降。

### 12.2 交通灯和停止线

红灯场景应观察：

- 停止线前的 trajectory velocity 是否平滑下降。
- `stop_reasons` 是否指出 traffic light/stop line。
- traffic signal 是否在 timeout 内。
- Velocity Smoother 是否因 jerk/减速度限制而提前开始减速。

若停止点正确但车辆减速过晚，优先查看速度平滑和上游时间戳；若停止点错误，优先查看 vector map 的停止线和 Behavior Velocity 模块。

### 12.3 静态障碍物

静态障碍物的典型表现有两种：

- 道路宽度足够：Behavior Path Planner 生成横向绕行 path。
- 道路宽度不足或避障未批准：Motion Velocity Planner 生成减速/停车约束。

默认 preset 开启静态障碍物绕行和 obstacle stop/slowdown，但动态障碍物横向避让默认关闭。因此“看见动态目标但没有横向绕行”不一定是故障，可能是 preset 的设计结果。

### 12.4 换道

换道可能经历：

```text
IDLE -> REQUESTING/PREPARING -> RUNNING -> SUCCESS
```

实际表现受目标车道间隙、RTC approval、前后目标速度、路口距离和 path 可行性影响。只改换道距离参数而不记录当前 module state，通常无法判断是“没有触发”还是“触发后等待批准”。

### 12.5 Parking

当车辆不在 lane 或目标位于停车区域时，Scenario Selector 可能选择 Parking。典型链路为：

```text
parking scenario
  -> costmap_generator/occupancy_grid
  -> freespace_planner/trajectory
  -> is_completed
  -> 回到 lane driving
```

停车轨迹可能包含多次前进/倒车和较低速度，不能用 lane driving 的“始终沿 lane centerline”标准判断其正确性。

## 13. 可操作的调试与验证

### 13.1 关键 topic 检查

```bash
ros2 topic list | grep '^/planning/scenario_planning'
ros2 topic info /planning/scenario_planning/scenario -v
ros2 topic info /planning/scenario_planning/scenario_selector/trajectory -v
ros2 topic info /planning/scenario_planning/lane_driving/trajectory -v
ros2 topic info /planning/scenario_planning/velocity_smoother/trajectory -v
ros2 topic echo /planning/scenario_planning/status/stop_reasons --once
ros2 topic echo /planning/scenario_planning/scenario --once
ros2 topic hz /planning/scenario_planning/velocity_smoother/trajectory
```

### 13.2 按规划层验证

| 验证问题 | 观察对象 |
| --- | --- |
| 场景是否选对 | `scenario`、parking completed、ego 是否在 lane |
| 行为 path 是否正确 | `path_with_lane_id`、lane id、候选/debug marker |
| 交通规则是否生效 | traffic signal、stop reason、停止点和速度 |
| 运动学是否可行 | smoother path、曲率、path optimizer trajectory |
| 障碍物是否触发限速 | dynamic objects、pointcloud、velocity factor、max velocity candidates |
| 速度是否可跟踪 | velocity smoother trajectory、acceleration、jerk |
| 最终输出是否被拒绝 | planning validator diagnostics 和 `/planning/trajectory` |

### 13.3 三组推荐实验

#### 实验 A：红灯停车

记录 traffic signal、stop line、behavior velocity path 和最终 trajectory，验证：

$$
v(s_{stop})\approx0
$$

并比较不同减速/jerk 参数下停车开始位置和最大减速度。

#### 实验 B：静态障碍物

在安全仿真场景中分别关闭 `launch_static_obstacle_avoidance` 和改变 obstacle stop 参数，比较：

- path 的横向偏移。
- obstacle stop 的停止点。
- route 是否保持不变。
- stop reason 与 velocity factor。

#### 实验 C：Parking 场景

比较车辆在 lane 内、停车场内和停车完成后的 scenario topic、parking trajectory 和 `is_completed`，确认 Selector 的切换不会产生空轨迹或错误场景。

### 13.4 参数是否生效

```bash
ros2 param list /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner
ros2 param list /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner planning_hz
```

插件参数可能属于主节点命名空间，而不是独立节点。若模块通过 `launch_modules` 动态加载，应同时检查主节点的参数值和启动日志。

## 14. 常见问题和根因

| 现象 | 首先检查 | 典型根因 |
| --- | --- | --- |
| 没有 scenario trajectory | Scenario Selector | 输入 trajectory 未 ready、场景未初始化或选择条件不满足 |
| lane driving path 为空 | Behavior Path | route、vector map、odometry 或 module manager 状态 |
| 车辆突然变道 | Behavior Path | lane-change/avoidance 模块、RTC、目标车道间隙 |
| 红灯不停 | Behavior Velocity | signal timeout、停止线 map、插件未加载 |
| 障碍物前不减速 | Motion Velocity | objects/pointcloud 时间戳、模块开关、车辆 footprint 参数 |
| path 抖动 | Path Smoother/Path Optimizer | 输入采样间隔、平滑权重、边界约束或定位抖动 |
| 轨迹速度跳变 | Velocity Smoother | max velocity、acceleration、jerk、外部限速合并 |
| 停车后不回 lane driving | Freespace/Scenario Selector | `is_completed`、车辆是否真正回到 lane |
| 上游正常但最终轨迹异常 | Planning Validator | trajectory checker、collision checker、历史轨迹处理策略 |

## 15. 架构优点和边界

### 优点

- 场景、行为、运动和速度职责分层，便于调试和替换。
- Scene Module plugin 机制允许按车型、地图和产品需求组合功能。
- 规则模块可以通过 stop reason、velocity factor 和 debug marker 解释行为。
- Lane driving 与 parking 有明确的场景边界。
- 统一 Velocity Smoother 使不同场景输出服从加速度和 jerk 约束。

### 边界

- Scenario Planning 依赖 Mission route 和地图质量，不能自己修复错误 route。
- Behavior Path 默认是规则/模块化系统，不是单一全局最优控制器。
- 动态对象预测错误会直接影响绕障、停车和路口行为。
- 速度约束多源合并后可能明显保守，需要追踪每个模块的 stop reason。
- Scenario Planning 生成轨迹，但最终安全性还需 Planning Validator 和 Control 层确认。
- 默认 preset 与实验模块不同；动态避障、sampling planner、部分 roundabout/occlusion 模块并非默认启用。

## 16. 总结

Autoware Scenario Planning 的数学和工程本质是**在道路拓扑、车辆几何、动态障碍物、交通规则和运动学约束之间做分层可行性求解**：

```text
场景选择：选择 lane driving / parking
行为路径：在可行驶区域中选择横向行为
规则速度：在停止线、信号和道路规则处设置速度约束
运动规划：平滑并满足曲率、边界和车辆运动学
障碍速度：根据障碍物和道路参与者追加 stop/slowdown
速度平滑：满足速度、加速度、jerk 连续性
输出验证：检查延迟、轨迹合法性和碰撞风险
```

分析实际问题时，先确定异常发生在 `scenario`、path 几何、速度约束、运动学轨迹还是最终 Validator 输出，再查看对应 topic、插件状态和参数。不要只根据最终 `/planning/trajectory` 猜测中间决策，因为同一个最终速度结果可能来自交通灯、障碍物、边界或速度平滑器的不同约束。