# Autoware 规划算法架构分析

本文基于本仓库当前版本（Autoware 1.7.1 工作区）的 launch、消息定义、规划包和参数文件整理，重点回答四个问题：规划由哪些层组成、一次规划周期如何流动、各层使用什么算法、出现问题时应从哪里定位。

> 本文描述的是代码中的实际装配关系。模块是否启用、参数文件是否被加载，以及 topic 名称是否被 remap，最终都应以当前使用的 preset 和 launch 展开结果为准。

## 1. 先给结论：规划是分层约束叠加

Autoware 的规划不是一个单一的“最短路径算法”，而是一条由任务路径、场景选择、行为决策、运动可行性、速度约束和输出验证组成的流水线：

```mermaid
flowchart TD
    GOAL[目标点 / 起点 / 重规划请求]
    MAP[Lanelet2 矢量地图]
    STATE[定位状态\nkinematic_state / acceleration]
    OBJ[动态对象、点云、占据栅格]
    TL[交通灯与基础设施状态]

    GOAL --> MP[Mission Planning\n路线与 LaneletRoute]
    MAP --> MP
    MP --> SS[Scenario Selector\nlane driving / parking]
    STATE --> SS

    SS --> BP[Behavior Path Planning\n车道、变道、绕障、目标段]
    MAP --> BP
    STATE --> BP
    OBJ --> BP
    BP --> BVP[Behavior Velocity Planning\n交通规则与场景速度]
    MAP --> BVP
    OBJ --> BVP
    TL --> BVP

    BVP --> MPATH[Motion Planning\n平滑、运动学可行路径]
    STATE --> MPATH
    OBJ --> MPATH
    MPATH --> MVP[Motion Velocity Planning\n停车、减速、限速、边界保护]
    MAP --> MVP
    OBJ --> MVP
    MVP --> VS[Velocity Smoother\n加速度/jerk/速度连续性]
    VS --> VAL[Planning Validator\n延迟、轨迹、碰撞检查]
    VAL --> TRAJ[/planning/trajectory\nTrajectory]
```

核心设计可以概括为：

- **Mission Planning 决定“走哪条路线”**，输出路线上的 Lanelet 段。
- **Behavior Path Planning 决定“沿路线采取什么横向行为”**，例如保持车道、变道、绕过静态障碍物。
- **Behavior Velocity Planning 决定“在道路规则和场景约束下如何减速或停车”**，例如红灯、人行横道、路口和盲区。
- **Motion Planning 把行为路径变成车辆可执行的几何轨迹**，处理平滑、曲率和运动学可行性。
- **Motion Velocity Planning 在轨迹上叠加障碍物相关的速度限制**，并生成最终带速度的轨迹。
- **Planning Validator 是输出边界**，负责发现无效轨迹，并按配置决定继续发布、复用上一条有效轨迹或生成软停车轨迹。

## 2. 代码入口和运行时边界

### 2.1 顶层启动关系

从整车启动开始，规划相关的调用链是：

```text
autoware.launch.xml
  └─ tier4_planning_component.launch.xml
       └─ tier4_planning_launch/launch/planning.launch.xml
            ├─ mission_planning/mission_planning.launch.xml
            ├─ scenario_planning/scenario_planning.launch.xml
            ├─ autoware_planning_validator
            └─ autoware_planning_evaluator
```

关键文件：

- `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml`
- `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/planning.launch.xml`

顶层 launch 通过 `launch_planning`、`planning_module_preset`、`vehicle_model`、`use_sim_time` 和 `is_simulation` 等参数控制规划是否启动以及使用哪一组参数。规划输入对象和点云默认分别来自：

```text
/perception/object_recognition/objects
/perception/obstacle_segmentation/pointcloud
```

规划模块通常采用 ROS 2 composable node，多个组件可以装载到 multithreaded container 中。这会影响进程级调试、线程调度和 intra-process 通信，但不改变 topic 层面的逻辑边界。

### 2.2 当前版本的迁移兼容点

`autoware.launch.xml` 还启动了一个临时 relay：

```text
/planning/trajectory
        └─ relay ─> /planning/scenario_planning/trajectory
```

实际规划 launch 中，Planning Validator 的输入是：

```text
/planning/scenario_planning/velocity_smoother/trajectory
```

Validator 的输出是最终的：

```text
/planning/trajectory
```

因此排查“谁发布了最终轨迹”时，应优先检查 Validator，而不是把 `scenario_planning/trajectory` 误认为唯一的最终输出。

## 3. 规划数据契约

### 3.1 路线：LaneletRoute

`autoware_planning_msgs/msg/LaneletRoute.msg` 的主要字段为：

```text
std_msgs/Header header
geometry_msgs/Pose start_pose
geometry_msgs/Pose goal_pose
autoware_planning_msgs/LaneletSegment[] segments
unique_identifier_msgs/UUID uuid
bool allow_modification
```

它表达的是地图上的路线段，而不是带时间和速度的轨迹。`segments` 中的 lanelet 关系为后续行为规划提供道路拓扑约束；`uuid` 用于识别路线版本，`allow_modification` 影响规划器能否对路线进行修改或重规划。

### 3.2 最终轨迹：Trajectory

`autoware_planning_msgs/msg/Trajectory.msg` 只有：

```text
std_msgs/Header header
autoware_planning_msgs/TrajectoryPoint[] points
```

真正的空间、时间和运动信息在 `TrajectoryPoint` 中，包括位置、姿态、纵向速度、加速度、曲率以及相关时间信息。规划层之间的常见区别是：

| 数据形态               | 主要含义                      | 典型生产者                         |
| ---------------------- | ----------------------------- | ---------------------------------- |
| `LaneletRoute`       | 拓扑路线和起终点              | Mission Planner                    |
| `PathWithLaneId`     | 几何路径、lane id、可行驶区域 | Behavior Path Planner              |
| `Trajectory`         | 带速度/加速度等运动信息的轨迹 | Motion Planning、Velocity Planning |
| `Trajectory`（最终） | 已通过输出检查的控制输入      | Planning Validator                 |

不要把 route、path 和 trajectory 混为一谈：route 解决拓扑选择，path 解决几何形状，trajectory 解决随时间运动。

## 4. 逐层分析

### 4.1 Mission Planning：路线选择与重规划

代表包：`src/universe/autoware_universe/planning/autoware_mission_planner_universe/`

输入通常包括地图、当前位姿、起点和目标点，输出 `/planning/mission_planning/route`。其职责包括：

1. 将目标 pose 关联到可行驶 lanelet。
2. 在 lanelet 图上搜索从起点到目标点的路线。
3. 判断车辆是否到达目标。
4. 发现路线失效时执行 reroute。
5. 处理路线修改、手动变道等上层请求。

路线搜索的核心不是连续空间轨迹优化，而是 Lanelet2 拓扑图上的可达性和代价搜索。随后行为规划会根据路线中的 lanelet、拓扑邻接和交通规则生成局部路径。

当前参数中与行为直接相关的例子：

- `reroute_time_threshold`：触发重规划的时间条件。
- `minimum_reroute_length`：重规划的最小路线长度条件。
- `check_footprint_inside_lanes`：检查车辆 footprint 是否位于车道内。
- `goal_angle_threshold_deg`、`arrival_check_*`：目标到达判定。

调试入口：

```bash
ros2 topic echo /planning/mission_planning/route --once
ros2 node info /planning/mission_planning/mission_planner
```

如果 route 为空或 lanelet 不正确，继续看下游 path 没有意义，应先检查地图、目标点坐标系、定位 pose 和路线服务调用。

### 4.2 Scenario Selector：选择行驶场景

代码入口：`src/universe/autoware_universe/planning/autoware_scenario_selector/`

当前场景至少包括：

- `lane_driving`：普通道路行驶，进入行为、运动和速度规划。
- `parking`：自由空间停车，使用 `autoware_freespace_planner` 及其规划算法。

Scenario Selector 同时接收 route、车辆状态、lane driving trajectory 和 parking trajectory，根据路线和停车完成状态输出当前场景及对应 trajectory。它解决的是“当前应该由哪类规划器接管”，不是在同一条 lanelet 路线上做局部路径优化。

### 4.3 Behavior Path Planning：横向行为与候选路径

代码入口：

```text
src/universe/autoware_universe/planning/behavior_path_planner/
```

启动配置位于：

```text
src/launcher/autoware_launch/tier4_universe_launch/
  tier4_planning_launch/launch/scenario_planning/
    lane_driving/behavior_planning/behavior_planning.launch.xml
```

该层围绕 Scene Module Manager 组织多个行为模块。当前 launch 会按开关装载模块管理器，代表性模块有：

- `LaneChangeLeftModuleManager`、`LaneChangeRightModuleManager`：换道准备、目标车道选择和安全间隙检查。
- `StaticObstacleAvoidanceModuleManager`：在保持道路可行驶性的前提下绕过静态障碍物。
- `DynamicObstacleAvoidanceModuleManager`：考虑动态障碍物的横向避让。
- `AvoidanceByLaneChangeModuleManager`：通过换道规避障碍物。
- `SamplingPlannerModuleManager`：采样候选路径并按代价选择。
- `GoalPlannerModuleManager`、`StartPlannerModuleManager`：目标段和起步阶段的特殊路径处理。
- `SideShiftModuleManager`、`BidirectionalTrafficModuleManager`：侧向偏移和双向交通场景。

典型处理过程：

```text
route + odometry + objects + map
  -> 当前 lanelet / 目标 lanelet 识别
  -> Scene Module 判断是否请求激活
  -> 生成一个或多个候选 path
  -> 碰撞、车道边界、间隙、舒适性和行为代价检查
  -> 选择/更新输出 path
```

这里的“算法”通常不是一个全局优化器，而是状态机、规则条件、候选路径生成和代价排序的组合。场景模块需要处理 `IDLE`、`RUNNING`、`SUCCESS`、`FAILURE` 等生命周期状态，避免多个模块同时修改同一段路径时发生不一致。

值得重点观察的约束：

- 车辆 footprint 与车道边界的几何关系。
- 与前后车的纵向距离、相对速度和时间间隙。
- 目标车道是否可达、是否允许跨越边界。
- 候选路径的曲率、横向偏移和变化率。
- RTC/approval 机制是否要求外部批准后才能执行变道。

输出通常是带 lane id 的 path，随后进入 Behavior Velocity Planner。行为路径的变化应通过 path topic、lane id、debug marker 和 scene module 状态共同判断。

### 4.4 Behavior Velocity Planning：交通规则速度规划

该层在已有行为路径上设置速度约束、停止点和减速区，不负责大范围改变路径几何。其输入除了 path、定位和动态对象，还包括交通灯、虚拟交通灯、占据栅格和点云。

当前 launch 以插件列表方式装载模块，代表性插件有：

- `TrafficLightModulePlugin`：根据交通灯状态和停止线生成减速/停车约束。
- `CrosswalkModulePlugin`、`WalkwayModulePlugin`：处理行人横道和人行区域。
- `IntersectionModulePlugin`、`RoundaboutModulePlugin`：路口、环岛的进入速度和先行权约束。
- `BlindSpotModulePlugin`、`DetectionAreaModulePlugin`：盲区和检测区域安全限制。
- `StopLineModulePlugin`：地图停止线约束。
- `OcclusionSpotModulePlugin`：遮挡区域的保守减速或停车。
- `SpeedBumpModulePlugin`：减速带速度限制。
- `NoStoppingAreaModulePlugin`、`NoDrivableLaneModulePlugin`：特殊区域行为限制。

多个插件可能同时给出速度上限或停止点，最终通常表现为更保守约束的叠加。工程上要区分两种变化：

- path 不变、速度曲线在停止线前下降：通常是 Behavior Velocity Planner 或后续速度规划。
- path 横向偏移或 lane id 改变：通常应回到 Behavior Path Planner 查找行为模块。

### 4.5 Motion Planning：平滑与运动学可行性

代码入口：

```text
src/universe/autoware_universe/planning/motion_velocity_planner/
src/universe/autoware_universe/planning/autoware_path_smoother/
src/universe/autoware_universe/planning/autoware_path_optimizer/
src/universe/autoware_universe/planning/sampling_based_planner/
```

当前 `motion_planning.launch.xml` 将运动层拆成几步：

1. **Path smoothing**：可选 `ElasticBandSmoother`，或者直接 relay 输入 path。
2. **几何路径生成**：可选 `PathOptimizer`、`PathSampler`，或者使用 Path-to-Trajectory 转换器。
3. **Motion Velocity Planner**：在生成的 trajectory 上叠加停车、减速和速度限制。
4. **Surround Obstacle Checker**：在启动时检查车辆周边障碍物，并发布速度限制/禁止起步原因。

#### Elastic Band Smoother

把离散路径视为受几何和障碍物约束的弹性带，通过平滑相邻点的形状改善曲率连续性。它适合在不大幅改变行为意图的前提下减少路径抖动，但其结果仍需经过车辆运动学和边界检查。

#### Path Optimizer

根据车辆参数、当前位置和路径约束生成运动学更可行的 path/trajectory。实现细节和代价项应以 `autoware_path_optimizer` 当前源码及参数为准，重点关注曲率、曲率变化率、横向边界以及前向可达性。

#### Path Sampler

采样多条几何候选路径，再通过碰撞和代价评价选择一条。它更适合需要在障碍物和道路边界之间搜索多种形状的场景，代价是候选数量、计算时间和参数敏感性增加。

### 4.6 Motion Velocity Planner：障碍物和运动级速度约束

该节点的插件由 launch 动态组装，当前包括：

- `ObstacleStopModule`：对静态障碍物生成停车约束。
- `ObstacleSlowDownModule`：在障碍物附近降低速度。
- `ObstacleCruiseModule`：根据障碍物和道路条件维持安全巡航速度。
- `DynamicObstacleStopModule`：针对动态障碍物做停止判断。
- `OutOfLaneModule`：车辆偏离可行驶区域时限制速度。
- `ObstacleVelocityLimiterModule`：发布速度上限候选。
- `RunOutModule`：处理可能突然进入车辆路径的道路参与者。
- `BoundaryDeparturePreventionModule`：防止轨迹接近道路边界或驶出边界。
- `RoadUserStopModule`：对道路参与者施加停止约束。

这些模块的共同模式是：读取同一条候选 trajectory 和环境状态，计算约束或修改 trajectory 的速度字段，同时发布 stop reason、velocity factor 或速度上限。它们通常不重新选择全局路线；需要改变横向绕行策略时，应该由行为路径层承担。

### 4.7 Velocity Smoother：使速度可执行

场景选择后的 trajectory 还会经过 `autoware_velocity_smoother`。它负责在最大速度、加速度、减速度和 jerk 等限制下平滑速度曲线，防止上游插件简单写入的速度约束造成跳变。

当前场景规划中的典型连接是：

```text
/planning/scenario_planning/scenario_selector/trajectory
  -> velocity_smoother
  -> /planning/scenario_planning/velocity_smoother/trajectory
```

外部速度限制、定位加速度、操作模式状态也会影响这一阶段。观察停车问题时，不能只看停止点，还要同时检查速度曲线、减速度、jerk 和车辆当前速度。

### 4.8 Planning Validator：规划输出的最后防线

Validator 的输入输出在 `planning.launch.xml` 中明确指定：

```text
input : /planning/scenario_planning/velocity_smoother/trajectory
output: /planning/trajectory
```

它至少覆盖三类检查：

1. **Latency Checker**：检查规划周期和消息是否过期。
2. **Trajectory Checker**：检查点间距、曲率、姿态跳变、横向/纵向加速度、jerk、转角和轨迹位移等。
3. **Collision Checker**：检查路口碰撞、后向碰撞等与环境相关的风险。

关键参数 `default_handling_type` 定义发现无效轨迹后的处理策略：

```text
0: 即使无效也发布当前轨迹
1: 发布上一条通过验证的轨迹
2: 发布上一条有效轨迹并生成软停车
```

这说明 Validator 默认不一定会静默丢弃轨迹。出现诊断错误时，要同时查看 handling type、连续错误计数阈值和 soft-stop 参数。

## 5. 一次规划周期的端到端工作流

以 lane driving 为例，可按以下顺序跟踪：

```text
1. 定位发布车辆 pose、速度和加速度
2. Mission Planner 根据 goal 和地图发布 LaneletRoute
3. Scenario Selector 选择 lane_driving
4. Behavior Path Planner 读取 route、map、objects，生成 PathWithLaneId
5. Behavior Velocity Planner 根据交通规则修改路径速度/停止点
6. Path Smoother 平滑路径
7. Path Optimizer 或 Path Sampler 生成运动学可行轨迹
8. Motion Velocity Planner 根据障碍物和边界增加停车/减速约束
9. Velocity Smoother 使速度满足 acceleration/jerk/velocity 限制
10. Planning Validator 检查轨迹并发布 /planning/trajectory
11. Trajectory Follower 读取最终轨迹并交给控制器
```

其中第 4 到第 9 步并不只是线性“传一条消息”：场景模块还会发布 debug marker、stop reason、velocity factor、approval 状态和速度上限候选，供其他规划组件和系统诊断使用。

## 6. 关键算法与约束的关系

| 问题                  | 主要层            | 常见方法/机制                           | 主要输出变化       |
| --------------------- | ----------------- | --------------------------------------- | ------------------ |
| 从起点到目标怎么走    | Mission           | Lanelet2 图搜索、路线可达性、reroute    | lanelet segments   |
| 是否变道              | Behavior Path     | 场景状态机、间隙检查、候选路径与代价    | path 几何、lane id |
| 是否绕过静态障碍物    | Behavior Path     | 横向偏移、候选 path、碰撞和边界约束     | path 几何          |
| 红灯/人行横道是否停车 | Behavior Velocity | 停止线、交通灯状态、行人和遮挡规则      | 停止点、速度       |
| 曲率是否可跟踪        | Motion Planning   | Elastic Band、优化、采样、运动学约束    | 几何形状、曲率     |
| 障碍物附近是否减速    | Motion Velocity   | stop/slowdown/cruise 插件、速度上限合并 | 速度、加速度       |
| 速度是否连续可执行    | Velocity Smoother | 速度、加速度、jerk 约束和分析式平滑     | 速度曲线           |
| 输出是否异常          | Validator         | 延迟、轨迹阈值、碰撞检查、历史轨迹回退  | 诊断、最终轨迹策略 |

规划架构的关键是把**决策约束**和**连续优化**分开：行为层决定意图，运动层负责可执行性，验证层负责拒绝明显不安全或不连续的结果。

## 7. 交互接口速查

### 7.1 重要输入

| 输入                                                      | 用途                                  |
| --------------------------------------------------------- | ------------------------------------- |
| `/planning/mission_planning/route`                      | 行驶路线和 lanelet 拓扑               |
| `/localization/kinematic_state`                         | 当前 pose、速度和车辆运动状态         |
| `/localization/acceleration`                            | 速度平滑和运动级约束                  |
| `/map/vector_map`                                       | lanelet、停止线、路口、横道等语义地图 |
| `/perception/object_recognition/objects`                | 动态/静态目标检测与跟踪结果           |
| `/perception/obstacle_segmentation/pointcloud`          | 障碍物和道路边界相关点云              |
| `/perception/traffic_light_recognition/traffic_signals` | 交通灯识别结果                        |
| `/perception/occupancy_grid_map/map`                    | 占据栅格约束                          |
| `/system/operation_mode/state`                          | 操作模式和自动驾驶状态                |

### 7.2 重要中间和输出

| 输出                                                                | 用途                         |
| ------------------------------------------------------------------- | ---------------------------- |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | 行为层路径                   |
| `/planning/scenario_planning/lane_driving/trajectory`             | lane driving 运动层输出      |
| `/planning/scenario_planning/scenario_selector/trajectory`        | 场景选择后的轨迹             |
| `/planning/scenario_planning/velocity_smoother/trajectory`        | 速度平滑后的 Validator 输入  |
| `/planning/scenario_planning/status/stop_reasons`                 | 停车原因解释                 |
| `/planning/scenario_planning/max_velocity*`                       | 各组件给出的速度限制候选     |
| `/planning/trajectory`                                            | Validator 输出的最终规划轨迹 |

实际 topic 可能因 launch namespace 或 preset 改变，应使用 `ros2 topic info -v` 确认 publisher/subscriber 和消息类型。

## 8. 参数和模块装配

规划参数主要由 `tier4_planning_component.launch.xml` 集中传入，常见配置目录为：

```text
src/launcher/autoware_launch/autoware_launch/config/planning/
├── mission_planning/
├── scenario_planning/
│   ├── common/
│   ├── lane_driving/behavior_planning/
│   ├── lane_driving/motion_planning/
│   └── parking/
└── preset/
```

算法包自身的默认参数也值得阅读：

- `autoware_mission_planner_universe/config/mission_planner.param.yaml`
- `behavior_path_planner/config/behavior_path_planner.param.yaml`
- `behavior_velocity_planner/config/behavior_velocity_planner.param.yaml`
- `autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml`
- `planning_validator/autoware_planning_validator/config/planning_validator.param.yaml`
- `planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml`

特别注意 `autoware_trajectory_optimizer` 的 `plugin_names`：插件执行顺序本身就是算法行为的一部分。例如轨迹点修正、运动学可行性约束、QP 平滑、MPT 优化和速度优化的先后顺序会影响最终结果。不能只修改 `use_*` 开关而忽略插件列表。

需要区分“仓库中存在的规划包”和“当前 launch 实际装载的算法”：当前 lane-driving 的 `motion_planning.launch.xml` 直接装配的是 `autoware_path_smoother`、`autoware_path_optimizer` 或 `autoware_path_sampler`，以及 `autoware_motion_velocity_planner`。`autoware_trajectory_optimizer` 是否参与某条运行链，应继续沿当前 preset 和 launch 的参数引用确认，不能仅凭目录名判断。

## 9. 可操作的源码阅读与调试流程

### 9.1 推荐源码阅读顺序

```text
1. autoware_planning_msgs 的 Route/Path/Trajectory 消息
2. autoware.launch.xml 和 tier4_planning_component.launch.xml
3. planning.launch.xml
4. mission_planning.launch.xml 和 Mission Planner
5. scenario_planning.launch.xml 和 Scenario Selector
6. behavior_planning.launch.xml
7. Behavior Path Planner 的 Scene Module
8. Behavior Velocity Planner 的规则插件
9. motion_planning.launch.xml
10. Motion Velocity Planner 和 Velocity Smoother
11. Planning Validator
```

每个 package 建议按 `package.xml -> launch -> config -> include -> src -> test` 阅读。先确认节点如何被装配，再看算法实现，能避免把未启用的模块当成当前运行逻辑。

### 9.2 最小运行时检查

```bash
ros2 node list | grep planning
ros2 topic list | grep '^/planning'
ros2 topic info /planning/trajectory -v
ros2 topic echo /planning/mission_planning/route --once
ros2 topic echo /planning/trajectory --once
ros2 topic echo /planning/scenario_planning/status/stop_reasons --once
ros2 topic hz /planning/trajectory
```

检查结果至少应回答：

- route 是否存在、是否包含合理的 lanelet 段。
- behavior path 是否在可行驶区域内。
- trajectory 是否有足够的前向长度和合理的点间距。
- 停车点前的速度是否逐渐下降。
- Validator 是否发布诊断或复用上一条轨迹。

### 9.3 按现象定位

| 现象                   | 优先检查                                                                |
| ---------------------- | ----------------------------------------------------------------------- |
| 没有 route             | goal、定位 frame、Lanelet2 地图、Mission Planner service                |
| route 正常但 path 为空 | route lanelet、车辆 footprint、Behavior Path Planner 状态               |
| path 偏移或突然变道    | lane change/avoidance 模块、候选代价、RTC approval、动态目标            |
| path 正常但提前停车    | traffic light、crosswalk、intersection、occlusion、stop reason          |
| 轨迹速度跳变           | Motion Velocity Planner、Velocity Smoother、外部速度限制                |
| 轨迹曲率异常           | path smoother、path optimizer/sampler、车辆参数                         |
| 最终轨迹与上游不同     | Planning Validator handling type、trajectory checker、collision checker |
| 修改 YAML 没有效果     | 当前 preset 是否引用该文件、参数是否被更高层覆盖、运行节点参数          |

参数实际生效情况可用：

```bash
ros2 param list /<node_name>
ros2 param get /<node_name> <parameter_name>
```

### 9.4 调参实验规范

一次只改一个参数或一个有明确语义的参数组，并记录：原值、改后值、场景、地图、车辆模型、启动命令、topic 频率、轨迹变化和诊断结果。

建议的低风险实验：

1. 修改交通灯或人行横道参数，比较停止点和速度曲线。
2. 修改 lane change 的准备距离或安全距离，比较变道触发位置。
3. 修改障碍物减速参数，比较 `stop_reasons` 和速度上限。
4. 修改轨迹平滑权重，比较曲率、横向加速度和跟踪误差。

不要只看 RViz 的一条线；应同时保存 route、path、trajectory、stop reason、diagnostics 和参数 diff。

## 10. 设计上的优点与工程风险

### 优点

- 分层清晰：路线、行为、运动和验证各自承担不同职责。
- 插件化：行为模块、速度模块和运动模块可以按 launch 开关组合。
- 可解释性较好：stop reason、velocity factor、debug marker 能解释很多速度变化。
- 支持不同运行模式：lane driving、parking、仿真和实车可使用不同 preset。
- Validator 位于最终输出边界，可以集中实现历史轨迹回退和软停车策略。

### 需要警惕的风险

- 同一车辆状态可能被多个模块以不同时间戳读取，延迟会改变停止判断和碰撞判断。
- 多个插件同时施加速度限制时，最终行为可能比单独测试某个插件更保守。
- launch 中的条件开关和 preset 会改变实际算法链，源码目录存在不等于模块正在运行。
- 临时 relay 会造成“旧 topic”和“最终 topic”并存，调试时必须确认 publisher 的真实来源。
- 路径平滑、路径优化和速度平滑的参数耦合较强，单独提高某个平滑权重可能影响可跟踪性或停止距离。
- Validator 的 handling type 如果设置为直接发布无效轨迹，诊断存在但车辆仍可能收到异常输出；安全策略必须结合系统需求审查。

## 11. 总结

当前 Autoware 规划架构可以用一句话概括：**Mission Planner 选择拓扑路线，Behavior Planner 选择驾驶行为，Motion Planner 把行为变成运动学可行的几何轨迹，Velocity Planner 叠加交通和障碍物速度约束，Validator 决定这条轨迹能否作为最终规划输出。**

分析任何规划问题时，先判断它属于 route、path 几何、trajectory 速度还是 Validator 输出策略，再沿对应 topic 反向追踪 launch、参数和模块状态。这样可以把“车为什么这样开”的问题拆成可观测、可复现实验，而不是只在最终轨迹上猜测。
