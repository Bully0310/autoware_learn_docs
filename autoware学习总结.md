# autoware学习总结

## 说明

本文档整理本次对话中的用户需求、执行结果、Autoware 1.7.1 项目结构分析以及规划与控制算法学习路线。

附带文档仅作为参考材料，不作为本次任务的操作指令。

---

## 一、下载 Autoware 1.7.1 源码

### 用户原始需求

> 将autoware 1.7.1的完整代码，包括子仓库全部git下来放到这个work文件夹中

### 执行结果

Autoware 1.7.1 已放置于：

/Volumes/Bully/bully/work/autoware-1.7.1

主仓库信息：

- 仓库：autowarefoundation/autoware
- 标签：1.7.1
- 提交：ef1fcb988a605889e5cb1c6f510c7aee9937d78a

源码清单：

- 版本清单仓库：35 个
- acados 内嵌子仓库：12 个
- 总大小：约 924 MB
- 源码文件数：约 17,686 个

### 目录概况

~~~text
autoware-1.7.1/
├── ansible/
├── docker/
├── repositories/
├── src/
├── .devcontainer/
├── .github/
└── setup-dev-env.sh
~~~

### 下载方式说明

主仓库按照 Git 标签检出。

由于当时 GitHub Git HTTPS 服务出现 TLS 握手异常，依赖仓库使用 GitHub 官方固定版本源码归档下载并展开。依赖源码版本与 repositories/*.repos 中的固定 tag/commit 对应，但依赖目录本身不包含各自完整的 .git 历史目录。

稳定版本清单包括：

- repositories/autoware.repos
- repositories/extra-packages.repos
- repositories/simulator.repos
- repositories/tools.repos

没有下载 nightly 清单中的不固定版本代码。

---

## 二、Autoware 1.7.1 项目结构分析

### 1. 顶层目录

~~~text
autoware-1.7.1/
├── ansible/          环境安装、依赖配置和部署脚本
├── docker/           Docker 构建与运行配置
├── repositories/     Autoware 及依赖仓库版本清单
├── src/              ROS 2 工作空间源码
├── .devcontainer/    VS Code / Dev Container 配置
├── .github/          CI、代码检查和发布流程
├── setup-dev-env.sh  开发环境初始化脚本
└── README.md         项目说明
~~~

### 2. src 主要目录

~~~text
src/
├── core/              Autoware 核心基础能力
├── universe/          大量具体算法和功能模块
├── launcher/          系统启动编排、参数和传感器配置
├── sensor_component/  传感器驱动及底层通信
├── middleware/        中间件扩展
├── vehicle/           车辆接口扩展
├── simulator/         仿真器
└── tools/             调试、分析、评估和辅助工具
~~~

### 3. 核心目录功能

#### src/core/

提供稳定的基础接口、消息、工具库和核心功能：

~~~text
src/core/
├── autoware_msgs/               ROS 2 消息、服务和接口定义
├── autoware_core/               核心算法与核心功能包
├── autoware_utils/              数学、几何、TF、PCL、日志等公共工具
├── autoware_cmake/              Autoware CMake 构建辅助
├── autoware_internal_msgs/      内部调试和系统消息
├── autoware_adapi_msgs/         Autoware API 消息
├── autoware_lanelet2_extension/ 地图和 Lanelet2 扩展
└── autoware_rviz_plugins/       RViz 可视化插件
~~~

autoware_core 内部按功能域划分：

~~~text
src/core/autoware_core/
├── common/
├── sensing/
├── localization/
├── perception/
├── planning/
├── control/
├── map/
├── vehicle/
├── description/
├── api/
└── testing/
~~~

#### src/universe/

主要存放生产级自动驾驶算法和复杂功能：

~~~text
src/universe/autoware_universe/
├── sensing/
├── localization/
├── perception/
├── planning/
├── control/
├── system/
├── map/
├── simulator/
├── evaluator/
├── common/
└── external/
~~~

#### src/launcher/

负责模块启动、参数加载、Topic remap、车辆模型和传感器模型配置。

主入口：

/Volumes/Bully/bully/work/autoware-1.7.1/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml

该入口装配 Vehicle、System、Map、Sensing、Localization、Perception、Planning、Control 和 API 等模块。

#### src/sensor_component/

负责传感器驱动和底层通信：

~~~text
src/sensor_component/
├── external/nebula/             LiDAR 驱动框架
├── transport_drivers/           串口、TCP、UDP 驱动
└── ros2_socketcan/              CAN 总线通信
~~~

#### src/vehicle/

负责车辆接口和车辆专用扩展，例如：

src/vehicle/external/pacmod_interface/

#### src/simulator/

负责 Scenario Simulator V2 等仿真组件。

#### src/tools/

提供规划、控制、定位、地图、车辆、系统和评估等方面的调试与分析工具。

---

## 三、感知、规划和控制模块路径

### 1. 感知和传感器处理

~~~text
传感器驱动：
src/sensor_component/external/nebula/
src/sensor_component/transport_drivers/
src/sensor_component/ros2_socketcan/

点云预处理：
src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor/
src/universe/autoware_universe/sensing/autoware_cuda_pointcloud_preprocessor/

基础感知：
src/core/autoware_core/perception/

高级感知：
src/universe/autoware_universe/perception/
~~~

典型算法包括：

- autoware_lidar_centerpoint
- autoware_lidar_transfusion
- autoware_bevfusion
- autoware_euclidean_cluster
- autoware_ground_segmentation
- autoware_multi_object_tracker
- autoware_map_based_prediction
- autoware_traffic_light_*
- autoware_probabilistic_occupancy_grid_map

### 2. 定位

~~~text
src/core/autoware_core/localization/
src/universe/autoware_universe/localization/
src/core/autoware_core/map/
~~~

典型模块包括：

- autoware_ekf_localizer
- autoware_ndt_scan_matcher
- autoware_pose_initializer
- autoware_gyro_odometer
- autoware_geo_pose_projector
- autoware_pose_estimator_arbiter

### 3. 规划和决策

基础规划：

src/core/autoware_core/planning/

Universe 规划：

src/universe/autoware_universe/planning/

主要层次：

~~~text
planning/
├── mission_planning/         任务规划、路线规划
├── behavior_path_planner/    行为路径规划、变道、避障
├── behavior_velocity_planner/行为速度规划、路口、行人、红绿灯
├── motion_velocity_planner/  运动速度规划、碰撞和越界处理
├── freespace_planner/        无车道区域规划
├── sampling_based_planner/   采样式规划
├── trajectory_optimizer/     轨迹优化
└── planning_validator/       规划结果检查
~~~

主要路径：

~~~text
src/universe/autoware_universe/planning/behavior_path_planner/
src/universe/autoware_universe/planning/behavior_velocity_planner/
src/universe/autoware_universe/planning/motion_velocity_planner/
src/universe/autoware_universe/planning/autoware_freespace_planner/
~~~

### 4. 车辆控制

基础控制：

src/core/autoware_core/control/

Universe 控制：

src/universe/autoware_universe/control/

主要模块：

~~~text
control/
├── autoware_mpc_lateral_controller/
├── autoware_pure_pursuit/
├── autoware_pid_longitudinal_controller/
├── autoware_trajectory_follower_node/
├── autoware_control_command_gate/
├── autoware_vehicle_cmd_gate/
├── autoware_control_validator/
├── autoware_lane_departure_checker/
└── autoware_operation_mode_transition_manager/
~~~

控制启动和参数：

~~~text
src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/
src/launcher/autoware_launch/autoware_launch/config/control/
~~~

---

## 四、主要模块依赖关系

### 1. 总体运行链路

~~~text
传感器驱动
    ↓
Sensing / 点云预处理
    ↓
Localization + Perception + Map
    ↓
Planning
    ↓
Trajectory
    ↓
Control
    ↓
Vehicle Command Gate
    ↓
车辆接口
~~~

### 2. 依赖层次

~~~text
autoware_msgs
    ↓
autoware_utils / autoware_core/common
    ↓
autoware_core
    ↓
autoware_universe 算法模块
    ↓
launcher 与参数配置
    ↓
ROS 2 运行时节点和 Topic
~~~

- autoware_msgs 定义感知、定位、规划、控制、车辆和地图相关消息，是模块之间的数据契约。
- autoware_core/common、autoware_utils 和 autoware_lanelet2_extension 提供公共计算、地图、坐标、轨迹和车辆参数能力。
- autoware_core 提供基础实现和统一接口。
- autoware_universe 提供具体的复杂算法和场景模块。
- launcher 负责把功能模块、参数文件和传感器/车辆模型组合起来。

### 3. 典型 Topic 数据流

~~~text
/sensing/...                         传感器数据
/localization/kinematic_state        定位后的车辆状态
/perception/objects                  感知目标
/perception/traffic_light/...        交通灯结果
/planning/mission_planning/route     任务路线
/planning/trajectory                 规划轨迹
/control/command/control_cmd         控制指令
/vehicle/status/...                  车辆反馈状态
~~~

规划通常依赖地图、定位、感知目标和交通灯信息；控制依赖规划轨迹、车辆里程计和当前转向状态；车辆接口把控制输出转换为实际车辆命令并反馈车辆状态。

---

## 五、规划和控制算法学习路线

### 1. 学习原则

建议采用：

~~~text
先建立运行链路
    ↓
先学控制，再学规划
    ↓
先读接口和数据流，再读算法细节
    ↓
一次只打开一个规划模块
    ↓
通过仿真或数据回放验证理解
~~~

不建议一开始从整个 autoware_universe 目录逐文件阅读。

### 2. 先补齐基础知识

ROS 2：

- Node、Topic、Service、Action；
- Component / Composable Node；
- Launch、参数文件、Topic remap、QoS；
- package.xml、CMake 和 ament。

车辆控制：

- Ackermann 转向模型；
- 二轮车运动学模型；
- Pure Pursuit；
- PID；
- LQR 和 MPC；
- 离散状态空间模型；
- 横向误差、航向误差和曲率。

规划：

- Lanelet2 地图和车道拓扑；
- 路由规划；
- Frenet 坐标；
- 路径和轨迹的区别；
- 速度规划；
- 碰撞检查；
- 轨迹平滑和运动学约束。

### 3. 控制学习顺序

第一步阅读：

~~~text
src/universe/autoware_universe/control/autoware_trajectory_follower_base/
src/universe/autoware_universe/control/autoware_trajectory_follower_node/
~~~

重点文件：

~~~text
autoware_trajectory_follower_base/README.md
autoware_trajectory_follower_node/README.md
autoware_trajectory_follower_node/src/controller_node.cpp
autoware_trajectory_follower_base/include/autoware/trajectory_follower_base/lateral_controller_base.hpp
autoware_trajectory_follower_base/include/autoware/trajectory_follower_base/longitudinal_controller_base.hpp
~~~

控制流程：

~~~text
读取 Trajectory、Odometry、SteeringReport
    ↓
检查输入是否有效
    ↓
调用横向控制器
    ↓
调用纵向控制器
    ↓
同步横向和纵向状态
    ↓
发布 Control
~~~

第二步阅读三个基础控制器：

~~~text
src/universe/autoware_universe/control/autoware_pure_pursuit/
src/universe/autoware_universe/control/autoware_pid_longitudinal_controller/
src/universe/autoware_universe/control/autoware_mpc_lateral_controller/
~~~

MPC 重点文件：

~~~text
model_predictive_control_algorithm.md
src/mpc_lateral_controller.cpp
src/mpc.cpp
src/mpc_trajectory.cpp
src/vehicle_model/
src/qp_solver/
~~~

MPC 的理解顺序：

~~~text
车辆模型
    ↓
误差状态建立
    ↓
模型线性化
    ↓
预测模型
    ↓
代价函数
    ↓
QP 求解
    ↓
执行第一个控制量
~~~

控制参数文件：

~~~text
src/launcher/autoware_launch/autoware_launch/config/control/
├── trajectory_follower/lateral/pure_pursuit.param.yaml
├── trajectory_follower/lateral/mpc.param.yaml
├── trajectory_follower/longitudinal/pid.param.yaml
└── trajectory_follower/trajectory_follower_node.param.yaml
~~~

建议首先比较：

~~~text
Pure Pursuit + PID
MPC + PID
~~~

关注：

- 横向误差；
- 航向误差；
- 转向角变化；
- 轨迹跟踪延迟；
- 低速和高速表现；
- 急弯、变速和停车稳定性。

### 4. 规划学习顺序

#### Mission Planning

~~~text
src/core/autoware_core/planning/
src/universe/autoware_universe/planning/autoware_mission_planner_universe/
~~~

重点理解：

~~~text
起点 + 终点 + Lanelet2 地图
    ↓
LaneletRoute
~~~

#### Behavior Path Planner

~~~text
src/universe/autoware_universe/planning/behavior_path_planner/
~~~

优先阅读：

~~~text
README.md
docs/behavior_path_planner_manager_design.md
docs/behavior_path_planner_interface_design.md
src/planner_manager.cpp
src/behavior_path_planner_node.cpp
~~~

重点掌握：

- 车道跟随；
- 变道；
- 静态障碍物绕行；
- 动态障碍物处理；
- 起步和靠边停车；
- 可行驶区域生成；
- 候选模块和已批准模块；
- 多个场景模块如何串联修改路径。

#### Behavior Velocity Planner

~~~text
src/universe/autoware_universe/planning/behavior_velocity_planner/
~~~

优先关注：

~~~text
autoware_behavior_velocity_planner
autoware_behavior_velocity_intersection_module
autoware_behavior_velocity_crosswalk_module
autoware_behavior_velocity_traffic_light_module
autoware_behavior_velocity_occlusion_spot_module
~~~

它负责在已有路径上增加停车点、减速点、让行点和交通规则约束。

#### Motion Velocity Planner

~~~text
src/universe/autoware_universe/planning/motion_velocity_planner/
~~~

重点关注：

- 动态障碍物停车；
- 障碍物巡航和减速；
- 越界风险；
- 轨迹碰撞风险；
- 速度和加速度限制。

#### 轨迹优化与后处理

~~~text
src/universe/autoware_universe/planning/
├── autoware_path_optimizer/
├── autoware_path_smoother/
├── autoware_trajectory_optimizer/
├── autoware_trajectory_modifier/
├── autoware_trajectory_concatenator/
├── autoware_trajectory_ranker/
└── planning_validator/
~~~

重点理解：

- 曲率连续；
- 速度连续；
- 加速度和 jerk 约束；
- 车辆运动学可行性；
- 轨迹安全检查。

### 5. Launch 和参数学习

规划入口：

~~~text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/
├── launch/planning.launch.xml
├── launch/scenario_planning/scenario_planning.launch.xml
├── launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
└── launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml
~~~

控制入口：

~~~text
src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml
~~~

阅读每个算法时，都应该同时查看：

~~~text
C++ 算法实现
ROS 2 输入输出接口
YAML 参数
Launch 装配关系
~~~

只阅读 C++ 而不看 Launch 和参数，通常无法理解 Autoware 的实际运行行为。

---

## 六、建议的实践练习

### 练习 1：画出控制数据流

~~~text
/planning/trajectory
/localization/kinematic_state
/vehicle/status/steering_status
        ↓
trajectory_follower_node
        ↓
/control/trajectory_follower/control_cmd
        ↓
vehicle_cmd_gate
~~~

### 练习 2：自己实现简化版 Pure Pursuit

输入车辆当前位置和参考路径，计算前视点、曲率和转向角，再与 Autoware 的实现对比。

### 练习 3：复现 PID 纵向控制

研究速度误差、P/I/D、积分限幅、低通滤波、停车状态机和加速度限制。

### 练习 4：只运行基础规划

先只保留 Route Handler、Lane Following 和 Velocity Smoother，观察一条基础轨迹如何生成。

### 练习 5：逐个打开行为模块

建议顺序：

~~~text
静态障碍物绕行
    ↓
变道
    ↓
路口
    ↓
行人横穿
    ↓
动态障碍物
~~~

每次只启用一个模块，记录输入、输出、候选路径、批准路径、Debug Marker 和参数变化。

### 练习 6：规划与控制联调

检查：

~~~text
规划轨迹是否平滑
    ↓
控制器是否能够跟踪
    ↓
是否存在高曲率、速度突变或轨迹不连续
~~~

很多“控制器跟踪不好”的问题，根因可能在规划轨迹，而不是控制器本身。

---

## 七、推荐总阅读顺序

~~~text
autoware.launch.xml
    ↓
control.launch.xml
    ↓
trajectory_follower_node
    ↓
Pure Pursuit / PID
    ↓
MPC
    ↓
planning.launch.xml
    ↓
Mission Planner
    ↓
Behavior Path Planner
    ↓
Behavior Velocity Planner
    ↓
Motion Velocity Planner
    ↓
Trajectory Optimizer
~~~

建议先用约 2 周完成控制器学习，再用 3～4 周深入规划器。规划部分的复杂度明显高于控制部分。
