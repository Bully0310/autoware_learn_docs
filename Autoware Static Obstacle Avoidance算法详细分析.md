# Autoware Behavior Path Planner Static Obstacle Avoidance 算法详细分析

## 1. 模块定位

Static Obstacle Avoidance 是 Behavior Path Planner 中基于规则的静态障碍物避让模块。它不重新搜索完整道路，而是以已有参考路径为基线，通过横向偏移（shift）绕开障碍物，再用路径安全检查、道路边界和纵向速度控制验证并修正输出。

该模块主要针对停止或近似停止的车辆、行人、自行车等物体。正在运动的物体通常不进入静态避障主目标集合，但可能作为周围物体参与候选路径的预测碰撞检查。

主要实现：

- 场景主流程：[scene.cpp](../../src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/src/scene.cpp)
- 目标筛选、物体包络和几何工具：[utils.cpp](../../src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/src/utils.cpp)
- 数据结构：[data_structs.hpp](../../src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/include/autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp)
- 避让线生成：[shift_line_generator.cpp](../../src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/src/shift_line_generator.cpp)
- 默认参数：[static_obstacle_avoidance.param.yaml](../../src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/config/static_obstacle_avoidance.param.yaml)

## 2. 总体处理链路

```text
上一模块参考路径、地图、感知物体
        |
        v
获取当前车道和可行驶边界
        |
        v
构造 ObjectData 和物体包络
        |
        v
通用条件、类型、行为、位置和道路空间过滤
        |
        v
判断是否必须避让、是否可停车、是否可横向绕行
        |
        v
ShiftLineGenerator 生成候选横向偏移线
        |
        v
PathShifter 生成 spline/linear 候选路径
        |
        v
检查路径有效性、舒适性和预测碰撞安全性
        |
        v
选择避让、让行、减速、停车或继续原路径
```

`updateData()` 的主要顺序是：

```cpp
fillFundamentalData();
fillShiftLine();
fillEgoStatus();
fillDebugData();
updateRTCData();
```

## 3. 当前路径和可行驶区域

### 3.1 当前车道

`fillFundamentalData()` 使用上一模块的 `reference_path` 获取当前车道序列：

```cpp
data.current_lanelets =
  utils::static_obstacle_avoidance::getCurrentLanesFromPath(...);
```

同时获取延伸车道、最近车道和红灯车道。当前车道为空时，本周期不继续规划。

### 3.2 扩展车道和边界

模块生成两套区域：

- `drivable_lanes`：根据 `use_lane_type` 扩展的常规可行驶区域。
- `drivable_lanes_same_direction`：只扩展同方向车道，用于判断是否进入对向区域以及是否需要人工批准。

常规区域通过 `generateExpandedDrivableLanes()` 或 `generateNotExpandedDrivableLanes()` 生成，左右边界通过以下函数计算：

```cpp
data.left_bound = utils::calcBound(..., true);
data.right_bound = utils::calcBound(..., false);
```

边界可以根据参数考虑当前/相邻车道、道路肩部、导流线、路口和 freespace 区域。

如果 ego 仍在当前车道，且当前车道或下一车道存在红灯，红灯所在车道不进行相邻车道扩展，避免为了绕障而穿越不应穿越的停止区域。

## 4. 物体目标筛选

入口是 `fillAvoidanceTargetObjects()`。它先调用 `separateObjectsByPath()`，将物体分为目标路径走廊内物体和走廊外物体。走廊外物体进入 `other_objects`，后续仍可能参与安全检查或道路边界裁剪。

### 4.1 通用条件

`filterTargetObjects()` 首先执行 `isSatisfiedWithCommonCondition()`：

1. 物体类别必须在 `target_type` 配置中启用。
2. 正在移动的物体不进入静态避障主目标。
3. 物体不能超出后向检查距离。
4. 物体不能超出前向检测距离。
5. 物体不能越过目标点。
6. 不允许修改目标点时，物体还必须与目标点保持足够距离。

`fillLongitudinalAndLengthByClosestEnvelopeFootprint()` 根据物体包络和参考路径计算 `object.longitudinal`、`object.length` 和 `object.overhang_points`。

### 4.2 速度条件

物体速度模长为：

```cpp
std::hypot(twist.linear.x, twist.linear.y)
```

并与物体类别对应的 `moving_speed_threshold` 比较。超过阈值时标记为移动物体，静态避障主过滤会将其放入 `other_objects`；安全检查仍可以根据其预测轨迹进行碰撞判断。

`fillObjectMovingTime()` 维护 `stop_time`、`move_time`、`last_stop` 和 `last_move`，用于判断物体是否持续停止。

### 4.3 车辆类型

车辆物体会计算：

```cpp
o.behavior = getObjectBehavior(...);
o.is_on_ego_lane = isOnEgoLane(...);
o.is_within_intersection = isWithinIntersection(...);
o.shiftable_ratio = getShiftableRatio(...);
o.to_centerline = getDistanceToCenterline(...);
o.is_parking_violation = isParkingViolation(...);
o.is_parked = isParkedVehicle(...);
o.is_adjacent_lane_stop_vehicle = isAdjacentLaneStopVehicle(...);
```

车辆行为按相对车道方向分为：

- `NONE`：基本平行于 ego lane；
- `MERGING`：向 ego lane 合流；
- `DEVIATING`：从 ego lane 偏离。

随后执行 `isNoNeedAvoidanceBehavior()` 和 `isSatisfiedWithVehicleCondition()`。

### 4.4 行人和自行车

非车辆物体会执行 `isSatisfiedWithNonVehicleCondition()`。典型排除条件包括：位于人行横道附近、太靠近车道中心线、位于路口，或位于不适合从当前方向绕行的相邻车道、道路肩部或对向车道区域。

## 5. 物体包络构建

包络构建入口是：

```cpp
fillObjectEnvelopePolygon(
  object_data, stored_objects_, closest_pose, parameters_);
```

### 5.1 当前帧几何包络

`createEnvelopePolygon()` 的步骤是：

1. 将 `PredictedObject` 的 shape 转成二维 polygon。
2. 使用参考路径最近点的 yaw 建立路径局部坐标系。
3. 将物体 polygon 变换到路径坐标系。
4. 在路径坐标系中计算轴对齐 bounding box。
5. 将包围盒变回地图坐标系。
6. 按物体类型的 `envelope_buffer_margin` 对 polygon 做扩张。

这样得到的矩形长边沿道路方向，便于计算物体前后端以及相对路径的横向侵入。

包络 buffer 为：

```cpp
envelope_buffer_margin * object_data.distance_factor
```

默认配置中，车辆 buffer 通常为 0.1 m，行人和自行车通常为 0.5 m。

### 5.2 历史包络稳定化

`stored_objects_` 按 UUID 保存历史物体。若当前目标已有历史记录：

- 当前纵向误差椭圆过大时，优先复用历史包络，避免定位误差突然扩大导致包络跳变；
- 当前包络完全位于历史包络内时，继续使用历史包络；
- 当前包络超出历史包络时，对两者做 polygon union；
- 合并包络面积超过原始物体面积的 5 倍时，放弃累积结果，使用最新包络。

定位误差通过 `calcErrorEclipseLongRadius()` 计算，并与物体类型的 `th_error_eclipse_long_radius` 比较。

### 5.3 包络作用

包络用于计算物体纵向位置和长度、物体相对路径的最近 overhang 点、是否必须避让、横向避让距离、shift line 前后位置，以及优化型路径的障碍物 polygon。

## 6. 是否必须避让

`fillAvoidanceNecessity()` 使用物体包络最近 overhang 点，而不是物体中心点。

安全横向距离为：

```cpp
safety_margin =
  0.5 * vehicle_width
  + lateral_hard_margin * distance_factor;
```

停车车辆使用 `lateral_hard_margin_for_parked_vehicle`。当包络最近点侵入车辆安全范围时：

```cpp
object_data.avoid_required = true;
```

首次出现的物体使用普通阈值；如果历史上已经是 `avoid_required`，则使用 `hysteresis_factor_expand_rate`，避免物体在边界附近反复进入/退出避让状态。

需要区分：

- `target_objects`：经过筛选、需要关注的目标；
- `avoid_required`：不改变路径会侵入安全包络；
- `is_avoidable`：道路空间和策略是否允许绕行；
- `is_stoppable`：是否有足够距离停车；
- `isAbsolutelyNotAvoidable()`：当前是否完全无法通过该模块处理。

## 7. 道路空间和可绕行性

`getAvoidMargin()` 根据道路肩部距离和左右边界计算可用横向空间：

```cpp
hard_lateral_distance_limit =
  to_road_shoulder_distance
  - hard_drivable_bound_margin
  - 0.5 * vehicle_width;
```

如果硬边界内空间小于最小避让 margin，返回 `nullopt`，表示正常横向绕行空间不足。若软边界不足但硬边界尚可，则退化为最小 margin；正常情况下使用软边界限制后的最大 margin。

`isNoNeedAvoidanceBehavior()` 根据包络侵入和 `avoid_margin` 计算期望横移量：不需要横移时设置 `ENOUGH_LATERAL_DISTANCE`，横移小于 `lateral_execution_threshold` 时设置 `LESS_THAN_EXECUTION_THRESHOLD`，其他情况继续作为候选避障目标。

## 8. ShiftLine 和轨迹生成

### 8.1 生成 shift line

`fillShiftLine()` 调用：

```cpp
data.new_shift_line = generator_.generate(data, debug);
```

每条 `ShiftLine` 描述路径从当前横向位置到目标横向位置的变化，包括起点、终点、起止横向偏移量、路径索引和纵向距离。

多个障碍物的 shift line 会合并。若新方案更早开始避让，则删除与其冲突且开始位置更晚的旧 shift line。

### 8.2 横向运动约束

模块使用横向 jerk、横向加速度和当前道路速度计算 shift 时间，并配置 `PathShifter`：

```cpp
path_shifter.setVelocity(getEgoSpeed());
path_shifter.setLongitudinalAcceleration(...);
path_shifter.setLateralAccelerationLimit(...);
```

车辆不会瞬间横移，而是沿纵向距离逐渐完成横移，并在障碍物后逐渐回到参考路径。

### 8.3 路径有效性

`isValidShiftLine()` 检查：

1. 新路径在 ego 位置的横向偏移不能与当前实际偏移相差超过 0.1 m。
2. 车辆不能超出左右可行驶边界。
3. 检查中加入车辆半宽、硬边界 margin 和最大允许偏差。

### 8.4 生成 spline 和 linear 路径

```cpp
path_shifter_.generate(&spline_shift_path, true, SHIFT_TYPE::SPLINE);
path_shifter_.generate(&linear_shift_path, true, SHIFT_TYPE::LINEAR);
```

通常使用 spline 路径作为输出，linear 路径用于部分行为和转向灯判断。输出前会进行稀疏重采样。

## 9. 路径安全检查

`isSafePath()` 先判断从 ego 往前是否存在超过执行阈值的左移或右移。没有实际横移时直接认为安全。

有横移时调用 `getSafetyCheckTargetObjects()`。检查对象不仅包括主目标，也可能包括相邻区域物体、走廊外物体、后方物体和对向运动物体。

模块分别生成前方和后方 ego 预测路径，根据物体是否在前方、是否为对向运动选择对应预测轨迹；物体侧使用其预测路径，最终调用：

```cpp
checkCollision(
  shifted_path.path,
  ego_predicted_path,
  object,
  object_predicted_path,
  ...);
```

检查包含物体几何、预测路径、RSS 参数和航向差阈值。任意预测路径发生碰撞，候选路径即不安全。安全状态带有滞回，默认恢复安全需要连续 3 个周期通过检查。

## 10. 行为决策和速度控制

### 10.1 安全路径

```cpp
data.yield_required = false;
data.safe_shift_line = data.new_shift_line;
```

采用新的横向偏移路径。

### 10.2 不安全路径

如果允许 yield 且车辆尚未开始避让，则清除已批准的 shift line，进入让行/停车逻辑。若车辆已经在避让过程中，模块不会轻易重新切换到 yield，以避免行为抖动。RTC 强制激活或强制停用也会影响该决策。

### 10.3 纵向速度

- `insertPrepareVelocity()`：根据目标物距离、横向偏移、最小避让距离和横向 jerk 在进入横移前减速。
- `insertAvoidanceVelocity()`：在横向动作尚未完成前限制加速。
- `insertWaitPoint()`：尚未开始横移时停车等待。
- `insertStopPoint()`：已开始横移但路径不安全时，在车辆即将离开原始车道前减速。
- `insertReturnDeadLine()`：计算还能否安全回归原车道，必要时在回归截止点前减速。

## 11. 历史状态和目标丢失补偿

`updateStoredObjects()` 优先按 object UUID 匹配，UUID 不一致时再用 1.5 m 位置阈值匹配。未匹配目标累计 `lost_time`，超过 `object_last_seen_threshold` 后删除。

`compensateLostTargetObjects()` 会把历史保存中当前未检测到、且未被明确放入 `other_objects` 的目标重新加入 `target_objects`，防止感知短暂丢帧导致已开始的避让突然消失。默认丢失补偿时间为 2 秒。

新出现的 UNKNOWN 物体在 `unstable_classification_time` 内标记为分类不稳定，默认 2 秒，避免分类瞬态变化造成误避让。

## 12. 关键判断公式

### 12.1 是否进入主目标集合

```text
类型启用
且未超出纵向检测范围
且未超过目标点限制
且不是被排除的移动物体
且满足车型/行人行为规则
且包络与路径关系满足避让条件
```

### 12.2 是否必须避让

```text
包络最近 overhang 点
    侵入车辆半宽 + 类型 hard margin * distance_factor
```

则 `avoid_required = true`。

### 12.3 是否能绕行

```text
道路硬边界剩余空间
    >= 车辆半宽 + 最小横向安全 margin
```

否则不能正常横向绕行，可能转为减速或停车。

### 12.4 是否安全

```text
候选 shift path
与 ego/物体预测路径
在几何、RSS、航向差约束下无碰撞
```

## 13. 一句话总结

该模块先把感知物体转换为沿道路方向稳定的保守包络，再根据包络对参考路径的横向侵入和道路剩余空间决定“是否必须避让、是否能避让”，然后用满足横向运动学约束的 `ShiftLine + PathShifter` 生成平滑轨迹，最后通过预测碰撞检查和纵向减速/停车逻辑闭环完成行为决策。
