# Autoware Motion Planner 算法详细分析

本文基于当前工作区的 Motion Planning launch、`autoware_path_smoother`、`autoware_path_optimizer`、Path Sampler、`autoware_motion_velocity_planner` 和 `autoware_surround_obstacle_checker` 资料，分析 Autoware Universe lane driving 中 Motion Planner 的主要组成、数学原理、工作流程、参数、输入输出和实际表现。
\left(\arctan(L\kappa_k),-\delta_{max},\delta_{max}\right)
## 1. 核心定位

在 Scenario Planning 中，Behavior Path Planner 先决定车道、换道和绕障等行为，Motion Planner 再把行为 path 转换为车辆可以跟踪的 trajectory，并在轨迹上叠加障碍物和边界相关的速度约束。

当前代码中的 Motion Planning 不是一个单独算法，而是以下流水线：

```text
Behavior Path
  -> Path Smoother
  -> Path Optimizer 或 Path Sampler
  -> Motion Velocity Planner
  -> Lane Driving Trajectory
  -> Scenario Selector / Velocity Smoother / Validator
```

它主要解决：

- path 几何是否平滑。
- trajectory 是否满足车辆运动学。
- 车辆 footprint 是否在 drivable area 内。
- 优化后的轨迹是否与道路边界/障碍物冲突。
- 障碍物、越界和道路参与者是否需要减速或停车。

它不负责从起点到终点选择任务路线，也不负责决定红灯、换道等完整行为意图；这些由 Mission Planning 和 Behavior Planning 负责。

## 2. 实际运行架构

### 2.1 默认链路

当前默认 planning preset 选择：

```text
motion_path_smoother_type = elastic_band
motion_path_planner_type  = path_optimizer
```

因此默认主要链路为：

```mermaid
flowchart TD
    BP[Behavior Path Planner\nPathWithLaneId]
    EBS[Elastic Band Smoother]
    PO[Path Optimizer\nMPT/QP]
    MVP[Motion Velocity Planner]
    SUR[Surround Obstacle Checker]
    TRAJ[/lane_driving/trajectory]
    VS[Velocity Smoother]
    VAL[Planning Validator]

    BP --> EBS
    EBS --> PO
    PO --> MVP
    SUR --> MVP
    MVP --> TRAJ
    TRAJ --> VS
    VS --> VAL
```

### 2.2 Launch 装配

文件：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/
  scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml
```

这个 launch 根据参数选择：

1. `ElasticBandSmoother` 或 path relay。
2. `PathOptimizer`、`PathSampler` 或 Path-to-Trajectory converter。
3. `MotionVelocityPlannerNode` 及其 obstacle/boundary module。
4. 可选 `SurroundObstacleCheckerNode`。

多个组件装载在同一个 `motion_planning_container` 中，但它们仍通过明确的 topic remap 形成逻辑流水线。

## 3. 输入输出接口

### 3.1 Path Smoother/Path Optimizer 输入

| 输入 | 类型 | 用途 |
| --- | --- | --- |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | `PathWithLaneId`/内部 Path | 行为路径和 drivable area |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | ego pose、当前速度和最近轨迹点 |
| vehicle parameters | YAML 参数 | wheelbase、footprint、车辆尺寸和转向能力 |

### 3.2 Motion Velocity 输入

| 输入 | 用途 |
| --- | --- |
| `path_optimizer/trajectory` | 运动学优化后的输入轨迹 |
| `/map/vector_map` | 道路边界和语义地图 |
| `/localization/kinematic_state` | 当前车辆状态 |
| `/localization/acceleration` | 当前加速度和速度规划 |
| `/perception/object_recognition/objects` | 动态障碍物和道路参与者 |
| `/perception/obstacle_segmentation/pointcloud` | 无地面点云障碍物 |
| traffic/virtual traffic signals | 运动速度约束的辅助信息 |
| occupancy grid | 占据区域和障碍物表示 |

### 3.3 输出

| 输出 | 生产组件 | 含义 |
| --- | --- | --- |
| `path_smoother/path` | Path Smoother | 平滑后的行为路径 |
| `path_optimizer/trajectory` | Path Optimizer/Sampler | 位置和姿态可行的 trajectory |
| `/planning/scenario_planning/lane_driving/trajectory` | Motion Velocity Planner | 叠加障碍物/边界速度约束的 lane driving trajectory |
| `/planning/scenario_planning/max_velocity_candidates` | Motion Velocity/Surround Checker | 速度上限候选 |
| `/planning/scenario_planning/status/stop_reasons` | Motion Velocity | 停车原因 |
| `/planning/velocity_factors/motion_velocity_planner` | Motion Velocity | 速度影响因素和 debug factor |
| `/planning/scenario_planning/clear_velocity_limit` | Motion Velocity/Surround Checker | 清除速度限制命令 |

Path Optimizer README 明确指出：它只更新 trajectory 的 position/orientation，velocity 从输入 path 继承；最终速度处理由后续 Motion Velocity 和 Velocity Smoother 完成。

## 4. 单周期工作流程

```text
1. 接收 Behavior Path 和 ego odometry
2. 根据 path 采样间隔建立参考路径
3. Elastic Band 对 path 几何进行平滑
4. Path Optimizer/Sampler 生成运动学可行轨迹
5. 将输入 path 的速度插值到优化后的轨迹
6. 检查 trajectory footprint 是否越出 drivable area
7. 对障碍物、道路边界和道路参与者运行 Motion Velocity modules
8. 输出 stop/slowdown/cruise 速度约束
9. 生成 lane driving trajectory
10. 交给 Velocity Smoother 和 Planning Validator
```

优化并非每个周期都必然重新求解。Path Optimizer 会根据 ego 移动距离、goal 移动距离、path 形状变化和时间间隔判断是否需要 replan；否则复用上一条优化轨迹，仅更新速度。

## 5. Path Smoother：Elastic Band

### 5.1 作用

Elastic Band Smoother 的作用是减少 Behavior Path 中离散点、横向 shift 或局部避障带来的几何抖动，使 path 的曲率和采样连续性更适合后续优化器。

它不是一个独立的任务规划器，也不会决定换道目标车道；它主要在已有行为 path 附近做平滑。

### 5.2 数学抽象

给定离散参考点 $r_i$ 和待优化点 $p_i$，可以把弹性带平滑抽象为：

$$
\min_{p_0,\ldots,p_N}
\sum_{i=1}^{N-1}
\left[
\lambda_r\|p_i-r_i\|^2+
\lambda_s\|p_{i+1}-2p_i+p_{i-1}\|^2
\right]
$$

其中：

- 第一项保持对 Behavior Path 的跟踪。
- 第二项惩罚离散二阶差分，减少曲率变化。
- $\lambda_r$ 越大，越不愿意偏离行为 path。
- $\lambda_s$ 越大，输出越平滑，但可能牺牲避障形状。

真实实现还会结合车辆参数、odometry、边界和路径采样间隔。默认 `motion_path_smoother_type=none` 时，launch 直接 relay 输入 path，不进行平滑。

## 6. Path Optimizer：运动学可行轨迹

### 6.1 作用和特性

`autoware_path_optimizer` 的目标是：

- 生成运动学可行 trajectory。
- 尽可能使 trajectory 位于 drivable area 内。
- 检查车辆 footprint 与道路边界/障碍物的关系。
- 必要时在 trajectory 即将离开 drivable area 前插入零速度。

其优化输入是 path 和 drivable area，输出是 `Trajectory`。默认实现采用 Model Predictive Trajectory（MPT）思想和二次规划形式的优化。

### 6.2 Frenet 误差状态

车辆相对于参考路径的误差可以表示为：

```text
y       横向误差
theta   相对于参考路径的航向误差
delta   当前转角
```

在速度 $v$、时间步长 $dt$、轴距 $L$、参考曲率 $\kappa_k$ 下，车辆自行车模型在 Frenet 坐标中的离散形式为：

$$
y_{k+1}=y_k+v\sin(\theta_k)dt
$$

$$
\theta_{k+1}=\theta_k+
\frac{v\tan(\delta_k)}{L}dt-
\kappa_kv\cos(\theta_k)dt
$$

考虑转向执行器一阶延迟：

$$
\delta_{k+1}=\delta_k-
\frac{\delta_k-\delta_{des,k}}{\tau}dt
$$

其中 $\tau$ 是转向响应时间常数。

### 6.3 线性化

在 $\theta_k$ 较小时使用：

$$
\sin(\theta_k)\approx\theta_k
$$

参考转角由参考曲率得到，并限制在转角约束内：

$$
\delta_{ref,k}=\operatorname{clamp}
\left(\arctan(L\kappa_k),-delta_{max},\delta_{max}\right)
$$

令 $\delta_k=\delta_{ref,k}+\Delta\delta_k$，对 $\tan(\delta_k)$ 一阶展开后，系统写成：

$$
\mathbf x_{k+1}=A_k\mathbf x_k+\mathbf b_k u_k+\mathbf w_k
$$

其中：

$$
\mathbf x_k=[y_k,\theta_k]^T
$$

控制量可以取为转角或转角相关输入，$\mathbf w_k$ 表示参考曲率和线性化带来的常量项。

### 6.4 时间序列模型

把 $N$ 个状态和控制量堆叠后：

$$
\mathbf x=A\mathbf x_0+B\mathbf u+\mathbf w
$$

当路径不是从当前 ego pose 开始时，初始状态也可以作为优化变量：

$$
\mathbf u'=[\mathbf x_0^T,\mathbf u^T]^T
$$

这样可以处理 pull-over、goal 段或需要自由边界的路径优化。

## 7. Path Optimizer 的 QP/MPT 数学原理

### 7.1 目标函数

典型目标函数同时惩罚跟踪误差和控制变化：

$$
J_1=
w_y\sum_k y_k^2+
w_\theta\sum_k\theta_k^2+
w_\delta\sum_k\delta_k^2+
w_{\dot\delta}\sum_k\dot\delta_k^2+
w_{\ddot\delta}\sum_k\ddot\delta_k^2
$$

写成 QP 形式：

$$
J_1=\mathbf x'^TQ\mathbf x'+\mathbf u'^TR\mathbf u'
$$

代入状态方程后可写为：

$$
J(\mathbf u')=\mathbf u'^TH\mathbf u'+\mathbf u'^T\mathbf f+c
$$

其工程意义是：

- $w_y$ 大：更贴近 reference path。
- $w_\theta$ 大：更重视航向对齐。
- $w_\delta$ 大：减少转向输入。
- $w_{\dot\delta}$ 大：减少转向变化。
- terminal/goal 权重大：更重视轨迹末端位置和姿态。

### 7.2 车辆 footprint 近似

为降低优化复杂度，车辆 footprint 可以用多个圆近似：

$$
F_{vehicle}\approx\bigcup_{m=1}^{M}B(c_m,r_m)
$$

当前参数支持 `fitting_uniform_circle` 等圆集合方法。对每个圆心 $c_m$，优化器检查其相对于道路边界和障碍物的横向距离。

### 7.3 软碰撞/边界约束

对第 $k$ 个状态，边界条件可写成：

$$
y_{min,k}-\lambda_k\leq y_k\leq y_{max,k}+\lambda_k
$$

$$
\lambda_k\geq0
$$

将 slack 变量加入目标函数：

$$
J_{soft}=J_1+w_{soft}\sum_k\lambda_k^2
$$

当前设计把碰撞自由和道路边界作为软约束，而不是全部硬约束；优化失败或结果验证不通过时，会回退历史 trajectory。因此“QP 求解成功”不等于“轨迹一定无碰撞”，输出后仍必须做 drivable area/footprint validation。

### 7.4 Ego 附近的稳定性约束

车辆前方很近的 trajectory 如果每周期大幅变化，会导致方向盘抖动。MPT 对 ego 附近的历史 trajectory 保持约束，可抽象为：

$$
\mathbf x_k=\mathbf x_k^{prev},
\qquad k\in[0,N_{stable}]
$$

这是比碰撞软约束更强的局部稳定性要求。优化失败时，系统优先使用上一条有效 trajectory。

## 8. 重规划、历史轨迹和越界处理

### 8.1 Replan 条件

Path Optimizer 在以下情况之一满足时重新优化：

- ego 单周期移动超过 `replan.max_ego_moving_dist`。
- goal 位置移动超过 `replan.max_goal_moving_dist`。
- 输入 path 在 ego 周围横向变化超过阈值。
- 前方 path 形状变化超过阈值。
- 达到时间重规划条件。

否则，复用上一条优化 trajectory，并从最新输入 path 更新速度。

### 8.2 短段 MPT 与轨迹拼接

为降低计算成本，MPT 只优化 ego 前方有限长度的轨迹段，剩余 path 直接转换并拼接：

```text
优化段：MPT trajectory
剩余段：behavior path -> trajectory
最终：两段 concatenation
```

这意味着前方轨迹可能包含“优化段”和“直接转换段”两个不同来源；调试时不要把整条 trajectory 都当作 MPT 优化结果。

### 8.3 Drivable Area 外的处理

若优化 trajectory 进入 drivable area 外：

1. 有上一条有效 trajectory：使用上一条并在首次越界点前置零速度。
2. 没有上一条有效 trajectory：使用输入 path 的可行部分，并在越界前置零速度。
3. 若开启 `enable_outside_drivable_area_stop`，直接执行越界前停车逻辑。

设车辆 footprint 为 $F(s)$、可行驶区域为 $D$，则安全条件为：

$$
F(s)\subseteq D
$$

首次违反该条件的弧长位置为：

$$
s^*=\min\{s:F(s)\not\subseteq D\}
$$

输出轨迹在 $s^*$ 前应设置为可停止状态，而不是继续发布会越界的速度。

## 9. Path Sampler：可选采样规划

当 `motion_path_planner_type=path_sampler` 时，Path Sampler 接收平滑 path、odometry 和 objects，生成多条候选 path/trajectory，再按碰撞、边界、曲率和偏离代价选择：

$$
p^*=\arg\min_{p\in\mathcal P_{valid}}J(p)
$$

常见代价抽象为：

$$
J(p)=w_{ref}J_{reference}+
w_{curv}J_{curvature}+
w_{obs}J_{obstacle}+
w_{bound}J_{boundary}
$$

相比优化方法，采样可以覆盖多个局部形状；代价是候选数量增加时计算量增长。当前默认 preset 通常使用 Path Optimizer，Path Sampler 是可替换分支。

## 10. Motion Velocity Planner

### 10.1 作用

Motion Velocity Planner 接收 `path_optimizer/trajectory`，不主要改变横向几何，而是在轨迹上施加：

- 障碍物停车。
- 障碍物减速。
- 障碍物安全巡航速度。
- 动态障碍物停车。
- 越出车道时限速。
- 边界驶离预防。
- Run-out 道路参与者处理。
- Road-user stop。

模块通过 `motion_velocity_planner_launch_modules` 动态加载。

### 10.2 速度上限合并

在弧长位置 $s$ 上，设原始速度为 $v_{in}(s)$，各模块速度限制为 $v_i(s)$，则：

$$
v_{out}(s)=\min\left(v_{in}(s),v_1(s),v_2(s),\ldots,v_n(s)\right)
$$

若某模块要求停车：

$$
v_{out}(s_{stop})=0
$$

这就是多个 obstacle module 同时存在时速度趋于保守的原因。

### 10.3 障碍物停止距离

设障碍物沿轨迹距离为 $d_o$，当前速度为 $v$，系统延迟为 $T_d$，允许减速度为 $a_d>0$，安全 margin 为 $d_m$：

$$
d_{required}=vT_d+\frac{v^2}{2a_d}+d_m
$$

若：

$$
d_o\leq d_{required}
$$

则 Obstacle Stop 应插入停车约束；若距离更远但需要降低风险，则 Obstacle Slow Down 可设置较低的速度上限。

### 10.4 动态障碍物

对 ego 轨迹 $p_e(t)$ 和目标预测位置 $p_o(t)$，冲突条件可抽象为：

$$
\exists t\in[t_0,t_0+T],
\quad
\|p_e(t)-p_o(t)\|<d_{safe}(t)
$$

实际模块还会考虑目标 footprint、预测置信度、轨迹多边形和时间 margin。Motion Velocity 主要通过纵向减速/停车响应；横向绕行应由 Behavior Path Planner 处理。

### 10.5 轨迹多边形碰撞检查

Motion Velocity Planner 可将轨迹离散点扩展成车辆 polygon：

$$
P_{traj}=\bigcup_{i=0}^{N}F(p_i)
$$

点云或对象与轨迹 polygon 发生交叠时，可以触发 obstacle stop/slowdown。参数 `decimate_trajectory_step_length` 控制碰撞检查的纵向采样间隔，`goal_extended_trajectory_length` 控制目标附近额外轨迹长度。

## 11. Surround Obstacle Checker

`autoware_surround_obstacle_checker` 是 motion planning 的可选安全组件，重点不是检查整条 trajectory，而是车辆起步/停止状态下的近身障碍物。

### 11.1 输入输出

输入：

- `/perception/obstacle_segmentation/pointcloud`
- `/perception/object_recognition/objects`
- `/localization/kinematic_state`

输出：

- `max_velocity`：设置速度上限，通常可为零。
- `velocity_limit_clear_command`：清除限制。
- `no_start_reason`：禁止起步原因。
- footprint/debug markers。

### 11.2 状态机

```text
PASS
  -> 近身障碍物满足停止条件 -> STOP
STOP
  -> 障碍物离开恢复阈值并保持足够时间 -> PASS
```

设当前状态为 PASS 的触发距离为 $d_{entry}$，STOP 状态的恢复距离为 $d_{recover}$，通常：

$$
d_{recover}>d_{entry}
$$

形成距离迟滞，减少障碍物在阈值附近造成的 STOP/PASS 抖动。只有车辆速度低于 `stop_state_ego_speed` 并保持 `stop_state_entry_duration_time`，才进入停车相关检查。

## 12. 关键参数

### 12.1 Path Optimizer

文件：

```text
.../motion_planning/autoware_path_optimizer/path_optimizer.param.yaml
```

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `option.enable_skip_optimization` | `false` | 是否跳过 Elastic Band/MPT |
| `option.enable_outside_drivable_area_stop` | `true` | 越界前是否停车 |
| `common.output_delta_arc_length` | `0.5 m` | 输出 trajectory 采样间隔 |
| `common.output_backward_traj_length` | `5.0 m` | ego 后方轨迹长度 |
| `replan.max_path_shape_around_ego_lat_dist` | `2.0 m` | ego 附近 path 横向变化阈值 |
| `replan.max_path_shape_forward_lon_dist` | `100.0 m` | 前方 path 检查长度 |
| `replan.max_path_shape_forward_lat_dist` | `0.1 m` | 前方 path 横向变化阈值 |
| `replan.max_ego_moving_dist` | `5.0 m` | ego 移动触发重规划 |
| `replan.max_goal_moving_dist` | `15.0 m` | goal 移动触发重规划 |
| `mpt.common.num_points` | `100` | MPT 优化点数 |
| `mpt.common.delta_arc_length` | `1.0 m` | MPT 点间弧长 |
| `mpt.clearance.soft_clearance_from_road` | `0.1 m` | 软道路边界余量 |
| `mpt.weight.lat_error_weight` | `1.0` | 横向跟踪误差权重 |
| `mpt.weight.terminal_lat_error_weight` | `100.0` | 末端横向误差权重 |
| `mpt.weight.goal_lat_error_weight` | `1000.0` | 目标横向误差权重 |
| `mpt.weight.steer_input_weight` | `1.0` | 转向输入权重 |
| `mpt.weight.steer_rate_weight` | `1.0` | 转向变化权重 |
| `collision_free_constraints.option.soft_constraint` | `true` | 是否使用软碰撞约束 |
| `vehicle_circles` | 3 circles | footprint 圆近似数量/方法 |

### 12.2 Motion Velocity Planner

参数目录：

```text
.../motion_planning/motion_velocity_planner/
```

重点参数类别：

| 模块 | 参数关注点 |
| --- | --- |
| Obstacle Stop | 障碍物停止距离、目标过滤、停止 margin |
| Obstacle Slow Down | 减速区域、最低速度、横向/纵向 margin |
| Obstacle Cruise | 安全跟车和巡航速度 |
| Dynamic Obstacle Stop | 预测时间、动态目标冲突和停车 margin |
| Obstacle Velocity Limiter | 速度上限候选和清除条件 |
| Out of Lane | 越界判断、边界距离和限速 |
| Boundary Departure Prevention | 道路边界余量、速度限制和恢复条件 |
| Run Out/Road User Stop | 道路参与者类型、预测冲突和停止距离 |

### 12.3 Surround Obstacle Checker

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `surround_check_front_distance` | `0.5 m` | 前方近身障碍物距离 |
| `surround_check_side_distance` | `0.5 m` | 侧方近身障碍物距离 |
| `surround_check_back_distance` | `0.5 m` | 后方近身障碍物距离 |
| `surround_check_hysteresis_distance` | `0.3 m` | 恢复距离迟滞 |
| `state_clear_time` | `2.0 s` | 清除 STOP 状态时间 |
| `stop_state_ego_speed` | `0.1 m/s` | 判定车辆停止的速度阈值 |
| `stop_state_entry_duration_time` | `0.1 s` | 判定车辆停止的持续时间 |

## 13. 实际应用表现

### 13.1 正常道路跟踪

正常表现为：

```text
Behavior Path
  -> Elastic Band 减少 path 抖动
  -> MPT 生成可跟踪 trajectory
  -> 速度沿用并插值
  -> Motion Velocity 根据障碍物修改速度
```

车辆应保持平滑横向误差和有限转向变化。若 path 本身形状稳定但 trajectory 在 ego 前方抖动，优先检查 MPT 权重、历史轨迹复用和 replan 条件。

### 13.2 窄路和急弯

窄路表现高度依赖：

- `soft_clearance_from_road`。
- footprint 圆近似。
- `optimization_center_offset`。
- Path Optimizer 的 MPT 线性化范围。
- Behavior Path 的 drivable area。

优化器可能在狭窄道路无法找到满足所有软约束的解，随后回退上一条轨迹或在越界前停车。增大 clearance 有时会使窄路更安全，但也可能让可行解消失。

### 13.3 障碍物

常见表现：

- path 几何由 Behavior Path 改变，Motion Planner 只平滑和优化。
- path 不变，但 obstacle stop/slowdown 让速度下降。
- 优化后的 footprint 与障碍物冲突，Motion Velocity 进一步停车。
- 点云近身障碍物在车辆停止时触发 Surround Checker，阻止重新起步。

要判断是横向还是纵向决策，必须同时比较 `behavior_planning/path`、`path_optimizer/trajectory` 和 `lane_driving/trajectory`。

### 13.4 重规划和轨迹连续性

当 Behavior Path 发生大幅横向变化、ego 移动超过阈值或 goal 改变时，MPT 会重新求解；否则可能复用历史结果。实际效果是：

- 减少每周期重优化计算。
- 保持 ego 附近轨迹稳定。
- 但输入变化如果刚好低于阈值，轨迹可能短时间滞后于新的 path。

### 13.5 计算性能

MPT/优化方法相对采样方法计算量可控，但在复杂边界、障碍物多和约束紧的场景中可能变慢。优化点数、MPT 轨迹长度、replan 频率和 debug marker 发布都会影响实时性。

## 14. 可操作调试流程

### 14.1 Topic 检查

```bash
ros2 topic info /planning/scenario_planning/lane_driving/behavior_planning/path -v
ros2 topic info /planning/scenario_planning/lane_driving/motion_planning/path_smoother/path -v
ros2 topic info /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory -v
ros2 topic info /planning/scenario_planning/lane_driving/trajectory -v
ros2 topic echo /planning/scenario_planning/lane_driving/trajectory --once
ros2 topic echo /planning/scenario_planning/status/stop_reasons --once
ros2 topic hz /planning/scenario_planning/lane_driving/trajectory
```

### 14.2 按阶段定位

| 现象 | 优先检查 |
| --- | --- |
| 输入 path 已抖动 | Behavior Path Planner、定位、参考 lanelet |
| Smoother 后仍抖动 | Elastic Band 参数、输入采样间隔 |
| Optimizer 后横向偏离大 | MPT 权重、drivable area、clearance、optimization center |
| trajectory 突然回退 | MPT 求解失败、越界验证、历史 trajectory |
| 速度突然变为 0 | Motion Velocity stop module、Surround Checker、stop reason |
| path 正常但速度过低 | obstacle slowdown/cruise、外部速度限制、速度上限合并 |
| 窄路无法通过 | footprint 圆近似、soft clearance、MPT 可行性 |
| 重规划过频繁 | replan thresholds、path 形状噪声、goal 更新 |
| 计算超时 | MPT 点数、优化长度、replan 频率、debug 选项 |

### 14.3 参数检查

```bash
ros2 param list /planning/scenario_planning/lane_driving/motion_planning/path_optimizer
ros2 param list /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner
ros2 param get /planning/scenario_planning/lane_driving/motion_planning/path_optimizer mpt.common.num_points
ros2 param get /planning/scenario_planning/lane_driving/motion_planning/path_optimizer option.enable_outside_drivable_area_stop
```

如果实际节点名称或参数命名空间不同，以 `ros2 node list` 和启动日志为准。

### 14.4 三组推荐实验

#### 实验 A：窄路边界

改变 `soft_clearance_from_road` 和 `optimization_center_offset`，比较：

- MPT trajectory 与 drivable area 的距离。
- 轨迹是否越界。
- 是否在越界前置零速度。
- 优化耗时和历史轨迹回退。

#### 实验 B：输入 path 横向变化

在 Behavior Path 中改变一次 static avoidance 的 shift，观察 replan 条件是否触发，以及 ego 附近轨迹是否保持连续。记录 path shape difference、MPT output 和 steering command。

#### 实验 C：障碍物停止

分别改变静态障碍物距离、当前速度和 obstacle stop margin，验证：

$$
d_{required}=vT_d+\frac{v^2}{2a_d}+d_m
$$

是否能解释停止点变化，并比较 Motion Velocity 输出与最终 Velocity Smoother 输出。

## 15. 常见问题定位

| 现象 | 可能根因 |
| --- | --- |
| path_optimizer 没有输出 | 输入 path/odometry 未 ready、planner 类型或 container 未加载 |
| MPT 输出不稳定 | 输入 path 抖动、线性化误差、replan 频繁、转向权重不足 |
| 优化轨迹偏离参考 path | drivable area、障碍物约束、terminal/goal 权重或局部最优 |
| 窄路频繁停车 | soft clearance 太大、footprint 近似保守、MPT 无可行解 |
| 轨迹突然使用旧结果 | 优化失败或输出验证失败触发 fallback |
| 轨迹越界前未停车 | `enable_outside_drivable_area_stop` 未启用、drivable area 输入错误 |
| 障碍物前不减速 | objects/pointcloud 时间戳、Motion Velocity module 未加载 |
| 车辆起步被阻止 | Surround Obstacle Checker 处于 STOP、近身点云/对象未清除 |
| 速度已被修改但最终不同 | Velocity Smoother、外部限速或 Planning Validator 后处理 |

## 16. 设计优势和限制

### 优势

- 将几何平滑、运动学优化和速度约束分层，便于替换算法。
- MPT 使用线性化自行车模型，能够把车辆可跟踪性纳入优化。
- 历史轨迹和 ego 附近稳定约束改善控制连续性。
- drivable area 和 footprint 检查提供明确的越界保护。
- Motion Velocity 通过插件化 stop/slowdown/cruise 处理多类障碍物。

### 限制

- Path Optimizer 采用局部优化，非凸问题可能陷入局部最优。
- 碰撞和边界约束当前可使用软约束，求解成功不等于绝对无碰撞。
- 车辆模型、footprint 圆近似和线性化会影响窄路/急弯表现。
- Path Optimizer 与 Behavior Path 都可能承担局部避障，职责边界存在耦合。
- Motion Planner 不负责全局行为决策，输入 path 不合理时不应期待优化器替代行为规划。
- 计算复杂度和参数权重之间存在明显权衡。

## 17. 总结

Autoware Motion Planner 的核心链路是：

```text
行为 path
  -> 几何平滑
  -> Frenet 误差自行车模型
  -> QP/MPT 轨迹优化
  -> drivable area/footprint 验证
  -> 障碍物和边界速度限制
  -> lane driving trajectory
```

数学上，它把 path tracking、转向平滑、车辆运动学、道路边界和障碍物约束组合成局部优化问题；工程上，又通过历史 trajectory、replan 门限、越界停车和速度插件保证实时运行中的稳定性。

排查 Motion Planner 问题时，按以下顺序最有效：

```text
Behavior Path 是否合理
  -> Elastic Band 是否改变形状
  -> Path Optimizer 是否重规划/求解失败
  -> trajectory 是否越出 drivable area
  -> Motion Velocity 是否追加 stop/slowdown
  -> Velocity Smoother/Validator 是否再次修改
```