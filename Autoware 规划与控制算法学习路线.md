# Autoware 规划与控制算法学习路线

本文只针对当前工作区 `autoware-1.7.1`。源码版本以 [`repositories/autoware.repos`](../repositories/autoware.repos) 为准：`autoware_core` 为 `1.7.0`，`autoware_universe` 和 `autoware_launch` 为 `0.50.0`。路径、topic、launch 文件和参数文件均按该工作区编写。

适用背景：已了解 ROS 2 topic/service、熟悉 C++ 和 Python、了解规划与控制基本概念，但尚未系统阅读 Autoware 节点源码。目标是能读懂规划控制源码，并在仿真中修改参数、观察行为变化。

## 学习目标

完成路线后，应能验证以下能力：

1. 从 [`autoware.launch.xml`](../src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml) 追踪 Planning/Control 的节点、namespace、topic remap 和参数文件。
2. 解释 `Mission Planning`、`Behavior Path Planner`、`Behavior Velocity Planner`、`Motion Velocity Planner`、`Trajectory Follower` 之间的数据依赖。
3. 读懂一个 ROS 2 C++ node 的 publisher、subscription、timer、parameter、service 和 component 注册代码，并用 CLI 验证运行时连接。
4. 修改至少 5 个规划或控制参数，在 `planning_simulator.launch.xml` 中用 topic、RViz 或 rosbag 对比修改前后的轨迹、速度和控制指令。
5. 区分规划轨迹异常、控制跟踪误差、vehicle command gate 拦截、时间戳/TF/DDS 通信异常，并按固定顺序收集证据。

## 前置知识清单

### 必须掌握

- ROS 2：package、node、executor、topic、service、parameter、launch、namespace、remap、QoS。
- C++：类、继承、智能指针、STL、lambda、CMake；能读懂 `rclcpp::Node` 和 callback。
- Python：能读 launch 文件和简单数据分析脚本。
- 自动驾驶：车辆坐标系、地图坐标系、轨迹点、曲率、速度/加速度/加加速度、横向误差、航向误差。
- 控制理论：PID、Pure Pursuit、MPC、前馈/反馈、采样周期和延迟。
- 规划概念：route、lanelet、path、trajectory、behavior、velocity profile、collision check。

### 自测问题

1. `topic` 与 `service` 的通信模型分别是什么？为什么 `/planning/trajectory` 不适合用 service 传递？
2. ROS 2 的 relative topic name 如何经过 namespace 和 remap 变成完整名称？
3. 如何定义车辆相对参考轨迹的 lateral error 和 heading error？
4. Pure Pursuit 的 lookahead distance 增大时，转向通常如何变化？高速时为什么常需要速度相关的 lookahead？
5. PID 的积分项为什么可能 windup？应在哪里观察误差和输出限幅？
6. Path 与 Trajectory 的区别是什么？为什么同一条 path 还需要速度规划？
7. `frame_id`、时间戳和 TF 缺一项时，为什么会导致规划或控制结果异常？
8. `component_container_mt` 与独立进程的差异是什么？`use_intra_process_comms` 影响什么？

环境检查命令：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 pkg prefix autoware_launch
ros2 interface show autoware_planning_msgs/msg/Trajectory
ros2 interface show autoware_control_msgs/msg/Control
```

## 分阶段路线

### 阶段总览

| 阶段 | 主题                   | 主要源码                                                 | 验收结果                   | 预计耗时 |
| ---- | ---------------------- | -------------------------------------------------------- | -------------------------- | -------- |
| 1    | ROS 2 运行图与消息契约 | `autoware_launch`、`autoware_msgs`                   | 能解释 topic 链路          | 1-2 天   |
| 2    | Planning 总体链路      | Mission/Scenario/Motion Planning                         | 能追踪 route 到 trajectory | 3-4 天   |
| 3    | Behavior 与速度规划    | `behavior_path_planner`、`behavior_velocity_planner` | 能观察变道和减速           | 4-6 天   |
| 4    | Control 与轨迹跟踪     | `trajectory_follower_node`、控制器、gate               | 能解释跟踪误差             | 3-5 天   |
| 5    | 联调、调参与回归       | planning + control + simulator                           | 完成参数实验记录           | 4-7 天   |

### 阶段 1：建立 ROS 2 运行图和消息契约

#### 阶段目标

- 能从 launch 找到节点、容器、namespace、remap 和参数来源。
- 能读懂规划与控制消息字段。
- 能使用 CLI 观察一个运行中的 Autoware 图。

#### 核心概念

- `Node`、`Publisher`、`Subscription`、`Service`、`Parameter`。
- launch 的 `<include>`、`<node>`、`<node_container>`、`<composable_node>`、`<remap>`。
- `Trajectory`、`TrajectoryPoint`、`Control`、`use_sim_time`、`/clock`、TF 和 QoS。

#### 关键源码路径

- `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml`
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml`
- `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/planning.launch.xml`
- `src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml`
- `src/core/autoware_msgs/autoware_planning_msgs/msg/Trajectory.msg`
- `src/core/autoware_msgs/autoware_control_msgs/msg/Control.msg`

#### 参数文件路径

- `src/launcher/autoware_launch/autoware_launch/config/vehicle/raw_vehicle_cmd_converter/raw_vehicle_cmd_converter.param.yaml`
- `src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/config/vehicle_cmd_gate.param.yaml`
- `src/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml`

#### 仿真验证任务

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/path/to/autoware_map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  use_sim_time:=true

ros2 node list
ros2 node info /planning/mission_planning/mission_planner
ros2 topic list | grep -E 'planning|control|localization|clock'
ros2 topic info /planning/trajectory -v
ros2 topic echo /planning/trajectory --once
ros2 topic echo /control/trajectory_follower/control_cmd --once
ros2 topic echo /clock --once
```

验收：指出 `/planning/trajectory` 的 publisher、`/control/trajectory_follower/control_cmd` 的 subscriber，以及两类消息的类型、frame 和时间字段。

### 阶段 2：理解 Planning 总体链路

#### 阶段目标

- 从 route/goal 追踪到最终轨迹。
- 区分 Mission Planning、Scenario Planning、Motion Planning 和 Planning Validator。
- 说明几何约束和速度约束分别在哪一层产生。

#### 核心概念

```text
route / goal
  -> Mission Planning
  -> Scenario Planning
     -> Behavior Path Planner
     -> Behavior Velocity Planner
     -> Motion Velocity Planner
  -> trajectory post-processing / optimizer
  -> Planning Validator
  -> /planning/trajectory
```

`Mission Planning` 处理路线和目标；`Behavior Path Planner` 处理车道、变道和绕障；`Behavior Velocity Planner` 处理交通规则和速度；`Motion Velocity Planner` 在路径上做运动级速度修正；`Planning Validator` 负责延迟、轨迹和碰撞检查。

#### 关键源码路径

- `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/planning.launch.xml`
- `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/mission_planning/mission_planning.launch.xml`
- `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/scenario_planning.launch.xml`
- `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving.launch.xml`
- `src/universe/autoware_universe/planning/autoware_mission_planner_universe/`
- `src/universe/autoware_universe/planning/behavior_path_planner/`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/`
- `src/universe/autoware_universe/planning/motion_velocity_planner/`
- `src/universe/autoware_universe/planning/planning_validator/`
- `src/core/autoware_msgs/autoware_planning_msgs/msg/LaneletRoute.msg`
- `src/core/autoware_msgs/autoware_planning_msgs/msg/Trajectory.msg`

#### 参数文件路径

- `src/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml`
- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/config/behavior_velocity_planner.param.yaml`
- `src/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml`
- `src/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml`

#### 仿真验证任务

```bash
ros2 topic list | grep '^/planning'
ros2 topic echo /planning/mission_planning/route --once
ros2 topic echo /planning/scenario_planning/behavior_path_planner/path --once
ros2 topic echo /planning/scenario_planning/velocity_smoother/trajectory --once
ros2 topic echo /planning/trajectory --once
ros2 topic hz /planning/trajectory
```

在仿真中设置目标点，记录 route 是否包含预期 lanelet、path 是否在可行驶区域、速度是否在停止线/障碍物附近降低，以及 validator 是否发布诊断。先读 `planning.launch.xml` 的 include/remap，再读 package 的 launch、node 类和 scene module。

### 阶段 3：深入 Behavior Path Planner 和速度规划

#### 阶段目标

- 解释一个 scene module 的生命周期、输入和输出。
- 用 RViz/topic 证实变道、绕障、交通灯停车或人行横道减速。
- 区分 path 几何变化和 velocity profile 变化。

#### 核心概念

- `SceneModuleManager`：管理 scene module 的激活、更新和输出。
- `PathWithLaneId`、candidate path、drivable area、lane change approval、RTC interface。
- traffic rule、stop line、obstacle slowdown、occlusion、crosswalk。
- path smoothing 与 velocity smoothing 的时序关系。

#### 关键源码路径

- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/`
- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/`
- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/`
- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/`
- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/`
- `src/universe/autoware_universe/planning/motion_velocity_planner/`
- `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml`

#### 参数文件路径

- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml`
- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml`
- `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/config/static_obstacle_avoidance.param.yaml`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml`
- `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml`

#### 仿真验证任务

```bash
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_velocity_planner/trajectory --once
ros2 topic echo /perception/object_recognition/objects --once
ros2 topic echo /planning/scenario_planning/behavior_velocity_planner/debug/markers --once
ros2 service list | grep -E 'lane|rtc|approval'
```

任务：将 `lane_change.param.yaml` 中一个安全距离或准备距离改为原值的约 `0.5` 倍，观察变道触发位置；将 `crosswalk.param.yaml` 中停止/减速参数改为更保守值，观察轨迹速度是否提前降低。一次只改一个参数，保存 YAML diff 和 rosbag。

### 阶段 4：深入 Control 和 Trajectory Follower

#### 阶段目标

- 从 `/planning/trajectory` 追踪到横向/纵向控制器和 vehicle command。
- 解释 Pure Pursuit、PID、MPC 的输入、输出、约束和参数入口。
- 用 lateral/longitudinal error、速度、转角和控制指令判断跟踪问题。

#### 核心概念

```text
/planning/trajectory + /localization/kinematic_state
  -> nearest point / error calculation
  -> lateral controller + longitudinal controller
  -> trajectory follower control command
  -> shift decider / vehicle command gate
  -> control command gate / vehicle interface
```

`Trajectory Follower` 连接轨迹、最近点搜索、车辆参数和控制器；横向控制输出转向，纵向控制输出加减速；`Vehicle Command Gate` 在自动、外部和紧急控制之间仲裁；AEB、collision detector、control validator 可能限制或阻止输出。

#### 关键源码路径

- `src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml`
- `src/universe/autoware_universe/control/autoware_trajectory_follower_node/`
- `src/universe/autoware_universe/control/autoware_trajectory_follower_base/`
- `src/universe/autoware_universe/control/autoware_pure_pursuit/`
- `src/universe/autoware_universe/control/autoware_mpc_lateral_controller/`
- `src/universe/autoware_universe/control/autoware_pid_longitudinal_controller/`
- `src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/`
- `src/universe/autoware_universe/control/autoware_control_command_gate/`
- `src/universe/autoware_universe/control/autoware_shift_decider/`
- `src/universe/autoware_universe/control/autoware_control_validator/`
- `src/universe/autoware_universe/control/autoware_autonomous_emergency_braking/`
- `src/core/autoware_msgs/autoware_control_msgs/msg/Control.msg`

#### 参数文件路径

- `src/universe/autoware_universe/control/autoware_trajectory_follower_node/param/trajectory_follower_node.param.yaml`
- `src/universe/autoware_universe/control/autoware_trajectory_follower_node/param/lateral/pure_pursuit.param.yaml`
- `src/universe/autoware_universe/control/autoware_trajectory_follower_node/param/lateral/mpc.param.yaml`
- `src/universe/autoware_universe/control/autoware_trajectory_follower_node/param/longitudinal/pid.param.yaml`
- `src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/config/vehicle_cmd_gate.param.yaml`
- `src/universe/autoware_universe/control/autoware_control_validator/config/control_validator.param.yaml`
- `src/universe/autoware_universe/control/autoware_autonomous_emergency_braking/config/autonomous_emergency_braking.param.yaml`

#### 仿真验证任务

```bash
ros2 topic echo /planning/trajectory --once
ros2 topic echo /localization/kinematic_state --once
ros2 topic echo /control/trajectory_follower/control_cmd --once
ros2 topic echo /control/command/control_cmd --once
ros2 topic echo /control/command/gear_cmd --once
ros2 topic echo /vehicle/status/steering_status --once
ros2 topic hz /control/trajectory_follower/control_cmd
ros2 topic info /control/command/control_cmd -v
```

先使用 Pure Pursuit，记录低速/高速的横向误差和转向角；切换 MPC，保持车辆模型和场景不变，比较转向平滑性；再修改 PID 增益，观察加速/制动响应和速度超调。确认 follower 输出到最终 command 的变化没有被 gate 或安全检查截断。

### 阶段 5：联调、调参与回归验证

#### 阶段目标

- 设计单变量实验，分别判断规划参数和控制参数的影响。
- 用 rosbag 和 topic 统计比较实验结果。
- 形成“现象 → 证据 → 假设 → 修改 → 回归”的闭环。

#### 核心概念

先验证 map、route、objects、kinematic state、clock、TF，再验证 path、velocity profile、trajectory、controller error，最后验证 control command、vehicle status 和安全 gate。固定地图、场景、车辆模型和仿真速度，一次只改一个参数组。

#### 关键源码路径

- `src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml`
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml`
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml`
- `src/universe/autoware_universe/planning/planning_validator/`
- `src/universe/autoware_universe/control/autoware_control_performance_analysis/`
- `src/universe/autoware_universe/evaluator/autoware_planning_evaluator/`
- `src/universe/autoware_universe/evaluator/autoware_control_evaluator/`

#### 参数文件路径

- `src/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml`
- `src/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml`
- `src/universe/autoware_universe/control/autoware_control_performance_analysis/config/control_performance_analysis.param.yaml`
- `src/universe/autoware_universe/control/autoware_collision_detector/config/collision_detector.param.yaml`
- `src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/planning.yaml`
- `src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/control.yaml`

#### 仿真验证任务

```bash
mkdir -p /tmp/autoware-route-control-baseline
ros2 bag record -o /tmp/autoware-route-control-baseline \
  /planning/trajectory \
  /planning/scenario_planning/velocity_smoother/trajectory \
  /localization/kinematic_state \
  /control/trajectory_follower/control_cmd \
  /control/command/control_cmd \
  /vehicle/status/steering_status

ros2 topic hz /planning/trajectory
ros2 topic hz /localization/kinematic_state
ros2 topic hz /control/trajectory_follower/control_cmd
ros2 topic echo /diagnostics --once
ros2 service list | grep -E 'engage|emergency|route|localization'
```

完成三组对比：改变 lane change/obstacle avoidance 参数并比较 path；改变 traffic light/crosswalk/intersection 参数并比较停止点和速度曲线；改变 Pure Pursuit lookahead 或 PID 增益并比较跟踪误差和转角变化。每组保存 YAML diff、启动命令、bag 路径、统计结果和结论。

## 源码阅读顺序

| 顺序 | 包/目录                                                                     | 职责                               | 阅读重点                             |
| ---- | --------------------------------------------------------------------------- | ---------------------------------- | ------------------------------------ |
| 1    | `src/core/autoware_msgs/autoware_planning_msgs`                           | route、path、trajectory 消息和服务 | 字段、时间戳、lane id、轨迹点        |
| 2    | `src/core/autoware_msgs/autoware_control_msgs`                            | 控制消息                           | steering、longitudinal、stamp        |
| 3    | `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` | 整车启动                           | 模块开关、全局参数、include          |
| 4    | `tier4_universe_launch/.../planning.launch.xml`                           | Planning 编排                      | namespace、输入输出、validator       |
| 5    | `autoware_mission_planner_universe`                                       | route 到任务路径                   | route service、lanelet、goal         |
| 6    | `behavior_path_planner`                                                   | 行为路径                           | scene module、candidate path、RTC    |
| 7    | `behavior_velocity_planner`                                               | 交通规则和速度约束                 | stop point、速度限制、模块状态       |
| 8    | `motion_velocity_planner`                                                 | 运动级速度修正                     | obstacle、boundary、速度平滑         |
| 9    | `autoware_trajectory_optimizer`                                           | 轨迹平滑和可行性                   | QP/MPT/kinematic feasibility         |
| 10   | `planning_validator`                                                      | 规划安全检查                       | latency、trajectory、collision       |
| 11   | `tier4_universe_launch/.../control.launch.xml`                            | Control 编排                       | controller mode、container、remap    |
| 12   | `autoware_trajectory_follower_node`                                       | 连接轨迹和控制器                   | nearest search、controller selection |
| 13   | `autoware_pure_pursuit` / `autoware_mpc_lateral_controller`             | 横向控制                           | lateral error、lookahead、MPC        |
| 14   | `autoware_pid_longitudinal_controller`                                    | 纵向控制                           | speed error、PID、限幅               |
| 15   | `autoware_vehicle_cmd_gate`                                               | 指令仲裁                           | auto/external/emergency、engage      |
| 16   | `autoware_control_validator` / AEB / collision detector                   | 控制安全                           | 报警、限制和输出截断                 |
| 17   | `autoware_*_evaluator`                                                    | 评估指标                           | tracking error、planning factor      |

每个包按以下顺序阅读：`package.xml` → `CMakeLists.txt` → `launch/` → `config/` 或 `param/` → `include/` → `src/` → `test/`。先弄清 node 如何启动和配置，再阅读算法函数。

## 调参实践清单

下表中的参数名需以当前 YAML 为准；先确认完整键名和当前 launch 是否加载该文件。

| 参数文件                                                                                                    | 调整对象                | 预期差异                                               | 观测对象             |
| ----------------------------------------------------------------------------------------------------------- | ----------------------- | ------------------------------------------------------ | -------------------- |
| `.../autoware_behavior_path_lane_change_module/config/lane_change.param.yaml`                             | 变道准备距离/安全距离   | 改小通常更晚触发或接受更小间隙；改大更保守             | behavior path、RViz  |
| `.../autoware_behavior_path_static_obstacle_avoidance_module/config/static_obstacle_avoidance.param.yaml` | 障碍物横向避让阈值      | 改大通常更早绕行或扩大避让区域                         | path、debug marker   |
| `.../autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml`                     | 交通灯停止/减速参数     | 更保守时更早减速，停止点距离增大                       | trajectory 速度      |
| `.../autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml`                             | 人行横道减速/停止       | 影响横道前减速范围和停车行为                           | trajectory 速度      |
| `.../autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml`                      | 轨迹平滑权重/约束       | 平滑权重增大通常降低曲率变化，但可能降低跟踪精度       | trajectory 曲率      |
| `.../autoware_trajectory_follower_node/param/lateral/pure_pursuit.param.yaml`                             | lookahead 距离/速度系数 | 增大通常更平滑但弯道切角可能变大；减小更敏捷但可能振荡 | control command      |
| `.../autoware_trajectory_follower_node/param/lateral/mpc.param.yaml`                                      | MPC 权重/预测参数       | 影响误差、转角变化率和控制平滑性                       | control command      |
| `.../autoware_trajectory_follower_node/param/longitudinal/pid.param.yaml`                                 | PID 增益/积分限幅       | 增大响应更快，但可能超调、振荡或 windup                | 速度、加速度         |
| `.../autoware_vehicle_cmd_gate/config/vehicle_cmd_gate.param.yaml`                                        | 门控阈值/超时           | 更严格时可能更早进入安全输出或拒绝过期指令             | command、diagnostics |

调参规则：一次只修改一个参数或一个明确参数组；记录原值、改后值、场景、速度范围、topic 频率和结果；先确认参数实际被 launch 加载，避免修改未使用的 YAML。

## 常见误区与排查方法

### 修改 YAML 后行为没有变化

可能是文件未被当前 launch include，或参数被更高层 launch 覆盖：

```bash
ros2 param list /<node_name>
ros2 param get /<node_name> <parameter_name>
```

从 `planning.launch.xml` 或 `control.launch.xml` 反向检查 `<param from="..."/>` 的真实路径。

### 把 path 问题当成控制器问题

```bash
ros2 topic echo /planning/scenario_planning/behavior_planning/path_with_lane_id --once
ros2 topic echo /planning/trajectory --once
ros2 topic echo /control/trajectory_follower/control_cmd --once
```

path 已偏离车道时先查 Behavior Path Planner；几何合理但速度异常时查 velocity planner；只有轨迹合理而车辆跟踪偏差大时才优先查 controller。

### 只看最终 control command

`vehicle_cmd_gate`、AEB、collision detector 和 operation mode manager 都可能改变最终命令：

```bash
ros2 topic echo /control/trajectory_follower/control_cmd --once
ros2 topic echo /control/command/control_cmd --once
ros2 topic echo /control/current_gate_mode --once
ros2 topic echo /system/operation_mode/state --once
```

### 忽略时间、TF 和仿真时钟

```bash
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /localization/kinematic_state --once
```

确认所有节点使用 `use_sim_time:=true`，并检查时间戳连续、frame 一致、TF 可查询。

### 混淆 component 和独立进程

检查 `control.launch.xml` 中的 `component_container_mt`、`composable_node` 和 `use_intra_process`。同一 container 关注 executor/thread 和 intra-process；跨进程或跨容器还要检查 DDS、`ROS_DOMAIN_ID`、RMW 和网络。

### 使用错误的 topic 名称

规划中可能存在 namespace 和临时 relay。以运行时图为准：

```bash
ros2 topic list | grep planning
ros2 topic info /planning/trajectory -v
ros2 node list | grep planning
```

### 同时修改规划和控制参数

这会破坏因果判断。固定场景后先只改 Planning，确认轨迹变化；恢复基线后只改 Control，最后再做联合实验。

## 推荐资源

### 官方文档

- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [Autoware Universe](https://github.com/autowarefoundation/autoware_universe)
- [Autoware Core](https://github.com/autowarefoundation/autoware_core)
- [Autoware Launch](https://github.com/autowarefoundation/autoware_launch)
- [ROS 2 Concepts](https://docs.ros.org/en/jazzy/Concepts.html)
- [ROS 2 Topics and Services](https://docs.ros.org/en/jazzy/Topics.html)
- [ROS 2 Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html)
- [Lanelet2 Documentation](https://m-naumann.github.io/Lanelet2/)

### 论文

- Paden et al., [A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles](https://doi.org/10.1109/TIV.2016.2578706)：规划与控制技术分类。
- Werling et al., [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://doi.org/10.1109/ROBOT.2010.5509799)：Frenet frame 轨迹生成。
- Dolgov et al., [Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments](https://doi.org/10.1109/IROS.2008.4651072)：搜索、代价与可行路径。
- Mayne et al., [Constrained model predictive control: Stability and optimality](https://doi.org/10.1016/S0005-1098(98)00129-5)：MPC 约束优化。
- Coulter, [Implementation of the Pure Pursuit Path Tracking Algorithm](https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/)：Pure Pursuit 推导与实现。

### 书籍

- Steven M. LaValle, *Planning Algorithms*：搜索、采样规划和规划建模。
- Rajesh Rajamani, *Vehicle Dynamics and Control*：车辆模型、横纵向控制和稳定性。
- Francesco Borrelli、Alberto Bemporad、Manfred Morari, *Predictive Control for Linear and Hybrid Systems*：MPC、约束和在线优化。
- Brian P. Gerkey et al., *Programming Robots with ROS 2*：ROS 2 node、launch、topic、service 和调试。

## 最终验收清单

- [ ] 能从 `autoware.launch.xml` 找到 Planning/Control 入口。
- [ ] 能解释 route、path、trajectory 和 control command 的差异。
- [ ] 能读懂至少一个 Planning scene module 和一个 controller 的核心 callback。
- [ ] 能使用 `ros2 topic info -v`、`ros2 node info`、`ros2 param get` 定位连接和参数问题。
- [ ] 已完成至少 5 个参数实验，并保存基线、修改、topic 记录和结论。
- [ ] 能在仿真中区分规划误差、控制误差、门控行为和时间/TF/DDS 问题。
