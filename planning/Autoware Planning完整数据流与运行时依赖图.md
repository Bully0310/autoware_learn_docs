# Autoware Planning 完整数据流与运行时依赖图

本文基于当前工作区 `src/universe/autoware_universe/planning`、`autoware_launch` planning launch 和各包的实际装配关系，给出 Planning 的数据流、运行时容器关系、插件依赖和可选分支。

## 1. 图例和边界

| 表示         | 含义                                               |
| ------------ | -------------------------------------------------- |
| 实线箭头     | ROS topic、消息或运行时数据流                      |
| 虚线箭头     | approval、参数、速度限制或横切关系                 |
| `plugin`   | 通常由主节点动态加载，不一定是独立 ROS node        |
| `optional` | 由 preset、launch 条件或新 planning framework 决定 |
| `library`  | 编译/算法库，通常不直接发布 topic                  |

本文将运行时 topic 依赖与编译/库依赖分开。package 的编译依赖不代表运行时一定存在 ROS topic。

## 2. 完整运行时数据流

```mermaid
flowchart TD
    GOAL([Goal / route API / checkpoints])
    MAP[(vector_map)]
    ODO[(kinematic_state)]
    ACC[(acceleration)]
    OBJ[(objects)]
    TRACK[(tracked_objects)]
    PCL[(obstacle pointcloud)]
    OCC[(occupancy_grid)]
    TLS[(traffic signals)]
    MODE[(operation mode)]
    TURN[(turn indicators)]

    subgraph MISSION[Mission Planning]
      MANUAL[manual_lane_change_handler]
      MP[MissionPlanner + RouteSelector]
      RH[route_handler library]
    end
    ROUTE[(mission_planning/route)]
    STATE[(mission_planning/state)]

    GOAL --> MANUAL
    GOAL --> MP
    MANUAL -. route request .-> MP
    MAP --> MP
    MAP --> RH
    ROUTE --> RH
    ODO --> MP
    MODE --> MP
    RH -. lanelet/routing queries .-> MP
    MP --> ROUTE
    MP --> STATE

    subgraph SCENARIO[Scenario Planning]
      SS[Scenario Selector]
      subgraph LANE[Lane Driving]
        BP[Behavior Path Planner\nmanager + path plugins]
        BV[Behavior Velocity Planner\nrule plugins]
        SM[Path Smoother optional]
        PO[Path Optimizer optional]
        PS[Path Sampler optional]
        MV[Motion Velocity Planner\nvelocity plugins]
        SUR[Surround Obstacle Checker optional]
      end
      subgraph PARK[Parking]
        CG[Costmap Generator]
        FS[Freespace Planner]
        FSA[Freespace Algorithms library]
      end
      EXT[External Velocity Limit Selector]
      VSM[Velocity Smoother]
      HAZ[Hazard Lights Selector]
    end

    ROUTE --> SS
    MAP --> SS
    ODO --> SS
    MODE --> SS
    SS -->|LaneDriving| BP
    SS -->|Parking| CG

    ROUTE --> BP
    MAP --> BP
    ODO --> BP
    ACC --> BP
    OBJ --> BP
    OCC --> BP
    TLS --> BP
    BP -. modified_goal / reroute .-> MP
    BP --> PATH[(behavior_planning/path_with_lane_id)]

    PATH --> BV
    MAP --> BV
    ODO --> BV
    ACC --> BV
    OBJ --> BV
    PCL --> BV
    OCC --> BV
    TLS --> BV
    BV --> BVPATH[(behavior_planning/path)]

    BVPATH --> SM
    SM --> PO
    SM --> PS
    PO --> MV
    PS --> MV
    BVPATH -. converter alternative .-> MV
    MAP --> PO
    ODO --> PO
    OBJ --> PS
    MAP --> MV
    ODO --> MV
    ACC --> MV
    OBJ --> MV
    PCL --> MV
    OCC --> MV
    MV --> LANE_TRAJ[(lane_driving/trajectory)]
    SUR -. velocity limit / no_start .-> MV
    ODO --> SUR
    OBJ --> SUR
    PCL --> SUR

    MAP --> CG
    OBJ --> CG
    PCL --> CG
    SS --> CG
    CG --> GRID[(parking/occupancy_grid)]
    GRID --> FS
    FSA -. algorithms .-> FS
    ROUTE --> FS
    ODO --> FS
    FS --> PARK_TRAJ[(parking/trajectory)]
    FS --> PARK_DONE[(parking/is_completed)]
    LANE_TRAJ --> SS
    PARK_TRAJ --> SS
    PARK_DONE --> SS
    SS --> SELECTED[(scenario_selector/trajectory)]

    EXT -. max_velocity .-> VSM
    SELECTED --> VSM
    ACC --> VSM
    MODE --> VSM
    VSM --> SMOOTHED[(velocity_smoother/trajectory)]
    HAZ --> HAZOUT[(turn/hazard command)]

    subgraph NEW[New Planning Framework optional]
      DP[Diffusion Planner\nTensorRT generator]
      CAND[(CandidateTrajectories)]
      TO[Trajectory Optimizer/Selector]
    end
    ODO --> DP
    ACC --> DP
    TRACK --> DP
    TLS --> DP
    MAP --> DP
    ROUTE --> DP
    TURN --> DP
    DP --> CAND
    CAND --> TO

    VAL[Planning Validator]
    CHECK[latency / trajectory / collision checkers]
    SMOOTHED --> VAL
    VAL --> CHECK
    OBJ --> VAL
    PCL --> VAL
    MAP --> VAL
    VAL --> FINAL[( /planning/trajectory )]
    TO -. optional alternative output .-> FINAL
```

## 3. 旧 Universe 主链 topic 图

```mermaid
flowchart LR
    R[/planning/mission_planning/route]
    BP[/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id]
    BV[/planning/scenario_planning/lane_driving/behavior_planning/path]
    SM[/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path]
    PO[/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory]
    MV[/planning/scenario_planning/lane_driving/trajectory]
    SEL[/planning/scenario_planning/scenario_selector/trajectory]
    VS[/planning/scenario_planning/velocity_smoother/trajectory]
    OUT[/planning/trajectory]
    R --> BP --> BV --> SM --> PO --> MV --> SEL --> VS --> OUT
```

实际变体：

- `motion_path_smoother_type=none` 时，`SM` 是 relay。
- `motion_path_planner_type=none` 时，`PO` 是 Path-to-Trajectory converter。
- `path_sampler` 可替代 `path_optimizer`。
- `planning_validator` 的输出是旧主链最终输出。

## 4. 停车分支

```mermaid
flowchart LR
    MAP[(vector_map)] --> CG[Costmap Generator]
    OBJ[(objects)] --> CG
    PCL[(no-ground pointcloud)] --> CG
    SC[(scenario)] --> CG
    CG --> GRID[(occupancy_grid)]
    GRID --> FS[Freespace Planner]
    ROUTE[(mission route)] --> FS
    ODO[(odometry)] --> FS
    FS --> PARK[(parking/trajectory)]
    FS --> DONE[(parking/is_completed)]
    PARK --> SS[Scenario Selector]
    DONE --> SS
```

编译依赖方向：

```text
autoware_freespace_planner -> autoware_freespace_planning_algorithms (library)
```

## 5. Diffusion/new framework 分支

该分支来自 `autoware_diffusion_planner/README.md` 的集成示例，不等同于旧默认链路。

```mermaid
flowchart TD
    ODO[(kinematic_state)]
    ACC[(acceleration)]
    TRACK[(tracked_objects)]
    TLS[(traffic_signals)]
    MAP[(vector_map)]
    ROUTE[(mission route)]
    TURN[(turn_indicators_report)]
    DP[DiffusionPlannerNode\npreprocess + TensorRT + postprocess]
    CAND[(planning/generator/diffusion_planner/\ncandidate_trajectories)]
    OPT[autoware_trajectory_optimizer]
    FINAL[(planning/trajectory)]
    ODO --> DP
    ACC --> DP
    TRACK --> DP
    TLS --> DP
    MAP --> DP
    ROUTE --> DP
    TURN --> DP
    DP --> CAND --> OPT --> FINAL
```

## 6. ROS 运行时容器图

```mermaid
flowchart TB
    subgraph NODES[独立节点/进程]
      MP[MissionPlanner]
      RS[RouteSelector]
      DP[DiffusionPlanner optional]
      EVAL[Planning Evaluator]
    end
    subgraph BPC[behavior_planning_container]
      BPP[BehaviorPathPlannerNode]
      BV[BehaviorVelocityPlannerNode]
      BPPM[Behavior Path plugins]
      BVM[Behavior Velocity plugins]
    end
    subgraph MPC[motion_planning_container]
      SM[ElasticBandSmoother optional]
      PO[PathOptimizer optional]
      PS[PathSampler optional]
      MV[MotionVelocityPlannerNode]
      MVM[Motion Velocity plugins]
      SUR[SurroundObstacleChecker optional]
      CONV[PathToTrajectory converter optional]
    end
    subgraph PARKC[parking_container]
      CG[CostmapGenerator]
      FS[FreespacePlanner]
    end
    subgraph VSC[velocity_smoother_container]
      VSM[VelocitySmootherNode]
    end
    subgraph VALC[planning validator container]
      VAL[PlanningValidator]
      VC[checker plugins]
    end

    BPP --> BPPM
    BV --> BVM
    MV --> MVM
    VAL --> VC
    MP --> RS
    MP --> BPP
    BPP --> BV
    BV --> SM
    SM --> PO
    SM --> PS
    PO --> MV
    PS --> MV
    CONV --> MV
    CG --> FS
    MV --> VSM
    FS --> RS
    RS --> VSM
    VSM --> VAL
    DP -. candidate trajectory .-> VAL
```

### 6.1 独立节点

- `MissionPlanner`：路线、RouteState、reroute。
- `RouteSelector`：normal/MRM route 仲裁。
- `DiffusionPlannerNode`：可选神经轨迹生成器。
- Planning Evaluator：规划质量评估。

### 6.2 Composable node

- Behavior Path/Velocity Planner。
- Elastic Band、Path Optimizer、Path Sampler。
- Motion Velocity Planner、Surround Obstacle Checker。
- Costmap Generator、Freespace Planner。
- Velocity Smoother、Planning Validator。

### 6.3 容器内 plugin

- Behavior Path：lane change、static/dynamic avoidance、goal/start、side shift、sampling、bidirectional traffic 等。
- Behavior Velocity：traffic light、crosswalk、intersection、blind spot、detection area、occlusion、speed bump、stop line 等。
- Motion Velocity：obstacle stop/slowdown/cruise、dynamic stop、run-out、boundary、out-of-lane、road-user stop。
- Validator：latency、trajectory、intersection collision、rear collision checker。

## 7. 完整功能包清单

### 7.1 顶层 Planning 包

| 包                                              | 归属                  | 运行时角色                     |
| ----------------------------------------------- | --------------------- | ------------------------------ |
| `autoware_mission_planner_universe`           | Mission               | node + Lanelet2 planner plugin |
| `autoware_manual_lane_change_handler`         | Mission               | route/lane-change handler      |
| `autoware_scenario_selector`                  | Scenario              | scenario arbitration node      |
| `autoware_costmap_generator`                  | Parking               | composable node                |
| `autoware_freespace_planner`                  | Parking               | composable node                |
| `autoware_freespace_planning_algorithms`      | Parking               | algorithm library              |
| `behavior_path_planner`                       | Behavior Path         | manager + plugins              |
| `behavior_velocity_planner`                   | Behavior Velocity     | manager + plugins              |
| `autoware_path_smoother`                      | Motion                | optional node                  |
| `autoware_path_optimizer`                     | Motion                | optional MPT/QP node           |
| `motion_velocity_planner`                     | Motion Velocity       | manager + plugins              |
| `autoware_surround_obstacle_checker`          | Motion Velocity       | optional node                  |
| `sampling_based_planner`                      | Motion                | sampling libraries/node        |
| `autoware_diffusion_planner`                  | New framework         | optional neural generator      |
| `autoware_external_velocity_limit_selector`   | Cross-cutting         | velocity limit node            |
| `autoware_hazard_lights_selector`             | Cross-cutting         | light selector node            |
| `autoware_rtc_interface`                      | Cross-cutting         | RTC library                    |
| `autoware_remaining_distance_time_calculator` | Cross-cutting         | optional metric node           |
| `planning_validator`                          | Output safety         | validator + checker plugins    |
| `autoware_trajectory_optimizer`               | Candidate/postprocess | optional optimizer/selector    |
| `autoware_trajectory_adapter`                 | Candidate/postprocess | trajectory adapter             |
| `autoware_trajectory_concatenator`            | Candidate/postprocess | trajectory continuity          |
| `autoware_trajectory_modifier`                | Candidate/postprocess | trajectory modifier            |
| `autoware_trajectory_ranker`                  | Candidate/postprocess | candidate ranking              |
| `autoware_trajectory_safety_filter`           | Candidate/postprocess | safety filtering               |
| `autoware_trajectory_traffic_rule_filter`     | Candidate/postprocess | traffic-rule filtering         |

### 7.2 Behavior Path 子包

```text
autoware_behavior_path_planner
autoware_behavior_path_planner_common
autoware_behavior_path_lane_change_module
autoware_behavior_path_static_obstacle_avoidance_module
autoware_behavior_path_dynamic_obstacle_avoidance_module
autoware_behavior_path_avoidance_by_lane_change_module
autoware_behavior_path_external_request_lane_change_module
autoware_behavior_path_goal_planner_module
autoware_behavior_path_start_planner_module
autoware_behavior_path_sampling_planner_module
autoware_behavior_path_side_shift_module
autoware_behavior_path_bidirectional_traffic_module
```

### 7.3 Behavior Velocity 子包

```text
autoware_behavior_velocity_planner
autoware_behavior_velocity_crosswalk_module
autoware_behavior_velocity_walkway_module
autoware_behavior_velocity_traffic_light_module
autoware_behavior_velocity_intersection_module
autoware_behavior_velocity_roundabout_module
autoware_behavior_velocity_blind_spot_module
autoware_behavior_velocity_detection_area_module
autoware_behavior_velocity_virtual_traffic_light_module
autoware_behavior_velocity_stop_line_module
autoware_behavior_velocity_occlusion_spot_module
autoware_behavior_velocity_speed_bump_module
autoware_behavior_velocity_no_stopping_area_module
autoware_behavior_velocity_no_drivable_lane_module
autoware_behavior_velocity_rtc_interface
autoware_behavior_velocity_template_module
```

### 7.4 Motion Velocity 子包

```text
autoware_motion_velocity_planner
autoware_motion_velocity_obstacle_stop_module
autoware_motion_velocity_obstacle_slow_down_module
autoware_motion_velocity_obstacle_cruise_module
autoware_motion_velocity_dynamic_obstacle_stop_module
autoware_motion_velocity_obstacle_velocity_limiter_module
autoware_motion_velocity_out_of_lane_module
autoware_motion_velocity_boundary_departure_prevention_module
autoware_motion_velocity_run_out_module
autoware_motion_velocity_road_user_stop_module
```

### 7.5 Validator 和 Sampling 子包

```text
autoware_planning_validator
autoware_planning_validator_latency_checker
autoware_planning_validator_trajectory_checker
autoware_planning_validator_intersection_collision_checker
autoware_planning_validator_rear_collision_checker
autoware_planning_validator_test_utils

autoware_sampler_common
autoware_bezier_sampler
autoware_frenet_planner
autoware_path_sampler
```

## 8. 编译/库依赖图

```mermaid
flowchart TD
    CORE[Autoware Core\nmsgs / route_handler / utils / velocity_smoother]
    MAPLIB[Lanelet2 / map utilities]
    ROS[ROS 2 / rclcpp / components / pluginlib]
    CUDA[CUDA / TensorRT / ONNX]
    MP[Mission Planner]
    BP[Behavior Path Framework]
    BV[Behavior Velocity Framework]
    FS[Freespace Planner]
    MOT[Path Smoother / Path Optimizer]
    SAMPLE[Sampling libraries]
    MV[Motion Velocity Framework]
    DP[Diffusion Planner]
    POST[Trajectory adapters/filters/ranker]
    VAL[Planning Validator]

    MP --> CORE
    MP --> MAPLIB
    MP --> ROS
    BP --> CORE
    BP --> MAPLIB
    BP --> ROS
    BV --> CORE
    BV --> MAPLIB
    BV --> ROS
    FS --> CORE
    FS --> MAPLIB
    FS --> ROS
    MOT --> CORE
    MOT --> ROS
    SAMPLE --> MOT
    MV --> CORE
    MV --> ROS
    DP --> CORE
    DP --> MAPLIB
    DP --> CUDA
    DP --> ROS
    POST --> CORE
    POST --> ROS
    VAL --> CORE
    VAL --> ROS
```

## 9. 配置和启用关系

```mermaid
flowchart LR
    PRESET[planning preset YAML]
    TOP[autoware.launch.xml]
    COMP[tier4_planning_component.launch.xml]
    PLAN[planning.launch.xml]
    SCEN[scenario_planning.launch.xml]
    LANE[lane_driving.launch.xml]
    BP[behavior_planning.launch.xml]
    MOT[motion_planning.launch.xml]
    PARK[parking.launch.xml]
    PLUG[launch_modules string]

    PRESET --> TOP --> COMP --> PLAN --> SCEN
    SCEN --> LANE
    SCEN --> PARK
    LANE --> BP
    LANE --> MOT
    PRESET -. module flags .-> BP
    PRESET -. planner/smoother type .-> MOT
    BP --> PLUG
    MOT --> PLUG
```

一个包/模块实际运行的条件通常是：

```text
包已构建
  AND 上层 launch 已 include
  AND launch 条件为 true
  AND preset 已打开
  AND plugin 出现在 launch_modules
  AND 所需输入 topic 已 ready
```

## 10. Topic 边界速查

| 边界                               | 上游                      | 下游                            | 主要数据                       |
| ---------------------------------- | ------------------------- | ------------------------------- | ------------------------------ |
| Mission -> Scenario                | Mission Planner           | Scenario Selector/Behavior Path | `LaneletRoute`               |
| Scenario -> Behavior               | Scenario Selector         | Behavior Path Planner           | `Scenario`、route            |
| Behavior Path -> Behavior Velocity | Behavior Path Planner     | Behavior Velocity Planner       | `PathWithLaneId`             |
| Behavior -> Motion                 | Behavior Velocity Planner | Smoother/Optimizer              | path + velocity constraints    |
| Motion Geometry -> Motion Velocity | Path Optimizer/Sampler    | Motion Velocity Planner         | `Trajectory`                 |
| Motion Velocity -> Scenario        | Motion Velocity Planner   | Scenario Selector               | lane-driving trajectory        |
| Parking -> Scenario                | Freespace Planner         | Scenario Selector               | parking trajectory + completed |
| Scenario -> Smoother               | Scenario Selector         | Velocity Smoother               | selected trajectory            |
| Smoother -> Validator              | Velocity Smoother         | Planning Validator              | smoothed trajectory            |
| Validator -> Control               | Planning Validator        | Control/Trajectory Follower     | `/planning/trajectory`       |
| Generator -> Selector              | Diffusion Planner         | Trajectory Optimizer/Selector   | `CandidateTrajectories`      |

## 11. 如何在运行时核对图

### 11.1 节点和组件

```bash
ros2 node list | grep -E 'planning|mission|scenario|behavior|motion|trajectory|validator'
ros2 component list
```

`ros2 node list` 只能证明独立节点/容器存在；plugin module 应查看主节点参数和日志。

### 11.2 Publisher/subscriber

```bash
ros2 topic info /planning/mission_planning/route -v
ros2 topic info /planning/scenario_planning/lane_driving/behavior_planning/path -v
ros2 topic info /planning/scenario_planning/lane_driving/trajectory -v
ros2 topic info /planning/scenario_planning/velocity_smoother/trajectory -v
ros2 topic info /planning/trajectory -v
```

### 11.3 Plugin 是否加载

```bash
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner launch_modules
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner launch_modules
ros2 param get /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner launch_modules
```

### 11.4 新 Diffusion 分支

```bash
ros2 topic info /planning/generator/diffusion_planner/candidate_trajectories -v
ros2 topic info /planning/generator/trajectory_optimizer/candidate_trajectories -v
ros2 topic echo /planning/trajectory --once
```

## 12. 总结

Autoware Planning 的完整运行时依赖可以归纳为：

```text
地图/定位/感知/目标
  -> Mission route
  -> Scenario arbitration
  -> Behavior path and rule velocity
  -> Motion geometry and obstacle velocity
  -> lane/parking trajectory selection
  -> velocity smoothing
  -> candidate filtering / validation
  -> final /planning/trajectory
```

当前工作区同时存在：

1. **旧 Universe 主链**：Mission -> Scenario -> Behavior -> Motion -> Velocity -> Validator。
2. **新 generator/selector 链**：Diffusion 或其他 generator -> `CandidateTrajectories` -> Trajectory Optimizer/Selector -> final trajectory。
3. **横切依赖**：RTC、外部限速、hazard lights、remaining distance/time、filters、ranker 和 diagnostics。

排查时先确认当前启用哪条链，再沿 topic、container、plugin 和 preset 逐层核对，避免把未启用的包、同容器插件和可选新框架误画成当前车辆必然执行的路径。
