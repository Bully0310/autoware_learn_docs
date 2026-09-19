# ROS 2 Humble 完整知识体系参考手册

> 本文以 **ROS 2 Humble Hawksbill** 为基准，面向具备 Linux 与 C++/Python 基础的机器人开发者。示例优先使用 Ubuntu 22.04、`rclcpp` 和 `rclpy`；不同平台、RMW 实现（ROS Middleware）或补丁版本可能存在细节差异。

## 目录

1. [核心概念](#1-核心概念)
2. [架构原理](#2-架构原理)
3. [安装与环境](#3-安装与环境)
4. [常用命令行工具](#4-常用命令行工具)
5. [创建功能包](#5-创建功能包)
6. [编程接口](#6-编程接口)
7. [通信机制详解](#7-通信机制详解)
8. [Launch 与参数](#8-launch-与参数)
9. [TF2 与坐标变换](#9-tf2-与坐标变换)
10. [可视化与调试工具](#10-可视化与调试工具)
11. [仿真与导航](#11-仿真与导航)
12. [进阶主题](#12-进阶主题)
13. [生态与工具链](#13-生态与工具链)
14. [常见问题与排查](#14-常见问题与排查)

---

## 1. 核心概念

### 1.1 ROS 2 的基本心智模型

ROS 2 不是一个单独的操作系统，而是一套用于机器人软件开发的通信、执行、构建、配置和工具生态。一个运行中的系统通常由多个进程组成：每个进程包含一个或多个节点（Node），节点通过接口交换数据。

```text
进程（Process）
└── 节点（Node）
    ├── 发布者（Publisher） ── Topic ──> 订阅者（Subscription）
    ├── 服务服务器（Service Server） <── Service ──> 服务客户端（Client）
    ├── 动作服务器（Action Server） <── Action ──> 动作客户端（Client）
    ├── 参数（Parameter）
    └── 定时器、回调、执行器（Executor）
```

### 1.2 术语和关系

| 概念 | 英文/缩写 | 特征 | 适用场景 |
|---|---|---|---|
| 节点 | Node | 逻辑功能单元，有名称和命名空间 | 传感器驱动、算法、控制器 |
| 话题 | Topic | 发布/订阅，多对多，异步、流式 | 图像、激光、里程计、轨迹 |
| 服务 | Service | 请求/响应，一次调用通常应快速完成 | 查询、触发配置、重置 |
| 动作 | Action | 目标、反馈、结果，可取消 | 导航、机械臂运动、长任务 |
| 消息 | Message | `.msg` 定义的数据结构 | Topic 的类型 |
| 服务接口 | Service interface | `.srv` 定义 request/response | Service 的类型 |
| 动作接口 | Action interface | `.action` 定义 goal/result/feedback | Action 的类型 |
| 参数 | Parameter | 节点拥有的键值配置，运行时可访问 | 频率、阈值、文件路径 |
| 名称 | Name | 节点、话题、服务、参数均有命名规则 | 命名空间、重映射 |
| TF | Transform | 坐标系之间的时变/静态变换 | 传感器、底盘、地图坐标 |

Topic 不保存请求者身份，也不保证订阅者一定存在；Service 适合短请求，不应阻塞数十秒；Action 通过反馈和取消机制支持长任务。三者最终都由 DDS（Data Distribution Service）承载，但 ROS 2 对 Action 做了更高层的协议封装。

### 1.3 名称、命名空间和重映射

相对名称会结合节点命名空间解析为完整名称：`image` 可能变成 `/robot/camera/image`。绝对名称以 `/` 开头，不再受当前命名空间影响。运行时可以重映射：

```bash
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=/robot/chatter -r __node:=talker_front
```

参数名也可通过 `-p` 设置；ROS 参数、环境变量和普通命令行参数不是同一层机制。

### 1.4 接口定义示例

```text
# Temperature.msg
std_msgs/Header header
float32 celsius
bool valid
```

```text
# SetTarget.srv
float64 x
float64 y
---
bool accepted
string message
```

```text
# Navigate.action
geometry_msgs/PoseStamped target
---
bool success
string message
---
float32 remaining_distance
```

### 1.5 ROS 2 与 ROS 1 的关键差异

| 方面 | ROS 1 | ROS 2 |
|---|---|---|
| 核心发现 | 通常依赖 ROS Master | DDS 分布式发现，无中心 Master |
| 通信 | TCPROS/UDPROS | DDS/RTPS，经 RMW 抽象 |
| 配置 | 参数服务器 | 参数属于节点 |
| 实时与生命周期 | 支持有限 | QoS、Lifecycle、执行器、组件更完整 |
| 构建 | `catkin` | `ament` + `colcon` |
| Python 客户端 | `rospy` | `rclpy` |
| C++ 客户端 | `roscpp` | `rclcpp` |
| 动作 | `actionlib` | ROS 2 Action API |

---

## 2. 架构原理

### 2.1 分层结构

```text
应用代码：rclcpp / rclpy / launch / ros2 CLI
        ↓
rcl：语言无关的 ROS Client Library API
        ↓
rmw：ROS Middleware 抽象层（rmw_fastrtps_cpp 等）
        ↓
DDS 实现：Fast DDS、Cyclone DDS、Connext DDS 等
        ↓
操作系统网络、共享内存或进程内通信
```

`rclcpp` 和 `rclpy` 提供面向开发者的接口；`rcl` 提供公共语义；`rmw` 隔离具体 DDS。可通过环境变量选择实现：

```bash
printenv RMW_IMPLEMENTATION
ros2 doctor --report
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2.2 DDS 与分布式发现

DDS 使用 Domain（域）隔离通信空间。节点创建 Publisher、Subscription、Service 或 Action 实体后，DDS 通过 RTPS（Real-Time Publish-Subscribe）协议广播或单播发现参与者、端点和 QoS。ROS 2 的图信息由节点和端点的发现结果推导而来，因此不需要中心 Master。

发现依赖网络组播、端口、防火墙和一致的 `ROS_DOMAIN_ID`：

```bash
export ROS_DOMAIN_ID=42
ros2 node list
```

同一局域网中的机器通常可以互通；跨网段、容器或 Wi-Fi 环境可能需要配置 DDS XML、发现服务器、组播路由或网络模式。

### 2.3 QoS（Quality of Service）

QoS 不是一个全局开关，而是每个 Publisher/Subscription 端点的通信契约。匹配时，订阅者提出的要求不能比发布者提供的能力更严格。

| 策略 | 常见值 | 作用 |
|---|---|---|
| Reliability | `reliable` / `best_effort` | 可靠重传，或尽力而为、低延迟 |
| Durability | `volatile` / `transient_local` | 是否让新订阅者收到发布者缓存的最后数据 |
| History | `keep_last` / `keep_all` | 保留固定深度或尽可能全部 |
| Depth | 正整数 | `keep_last` 的队列深度 |
| Deadline | 时间周期 | 期望消息更新的最大间隔 |
| Lifespan | 时间周期 | 消息过期时间 |
| Liveliness | `automatic` 等 | 判断发布者是否仍然存活 |

典型选择：传感器图像常用 `best_effort + volatile + keep_last`；控制指令常用 `reliable + volatile`；静态地图或配置可考虑 `transient_local`。QoS 不匹配时，节点可能都在运行但完全收不到数据。

### 2.4 进程、执行器和回调

节点可以独立进程运行，也可以作为组件装入同一个容器进程（Composition）。执行器（Executor）从一个或多个回调组（Callback Group）中取出可执行回调：

- `SingleThreadedExecutor`：简单、回调串行。
- `MultiThreadedExecutor`：允许并行，但必须处理数据竞争和回调组约束。
- `StaticSingleThreadedExecutor`：适合实体集合稳定的场景。
- 回调组可选 `MutuallyExclusive` 或 `Reentrant`。

避免在订阅回调中执行长时间阻塞操作；可使用定时器、线程、异步 Action 或专门的执行器。

### 2.5 生命周期（Lifecycle）

生命周期节点由 `unconfigured`、`inactive`、`active`、`finalized` 等状态组成，通过 configure、activate、deactivate、cleanup、shutdown 等转换管理资源。驱动和安全关键节点可在 active 之前完成参数校验、硬件连接和 publisher 激活。

---

## 3. 安装与环境

### 3.1 平台支持边界

| 平台 | Humble 典型方式 | 说明 |
|---|---|---|
| Ubuntu 22.04 amd64/arm64 | 官方 deb | 最推荐，使用 `apt` |
| Windows 10/11 | 官方二进制包/源码 | 适合桌面开发，Linux 命令和部分包不同 |
| macOS | 源码构建或 Docker/虚拟机 | 不应假定存在与 Ubuntu 等价的官方 deb |

机器人驱动、Gazebo、GPU 和实时部署通常优先选择 Ubuntu。安装前应固定发行版、架构、ROS 发行版和 RMW，避免混用不同发行版的环境。

### 3.2 Ubuntu 22.04 安装 Humble

```bash
sudo apt update && sudo apt install -y locales curl gnupg lsb-release software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

服务器或容器可安装 `ros-humble-ros-base`；需要 RViz2、rqt 等桌面工具则选择 `desktop`。

### 3.3 Windows 与 macOS 注意事项

Windows 应使用 ROS 2 官方安装说明对应的 Python、Visual Studio、DDS 和环境脚本；命令通常在 PowerShell 中运行，路径格式和编译器选项与 Linux 不同。macOS 通常采用源码构建，需准备 Xcode Command Line Tools、Python、CMake、编译器和系统依赖；Gazebo、RViz2、硬件驱动可能需要额外兼容性处理。若目标是稳定学习环境，推荐使用 Ubuntu 22.04 虚拟机、Docker 或远程 Linux 主机。

### 3.4 ROS 1 与 ROS 2 共存

不要在同一个 shell 中无序 source 多个发行版。推荐使用独立终端、容器或明确的包装脚本：

```bash
# ROS 2 Humble shell
source /opt/ros/humble/setup.bash

# ROS 1 Noetic shell（另开终端）
source /opt/ros/noetic/setup.bash
```

ROS 1/ROS 2 互通时使用 `ros1_bridge`，并确保两边消息定义可识别、网络和名称映射清晰。

### 3.5 工作空间与环境变量

```bash
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws
colcon build --symlink-install
source install/setup.bash
echo 'source ~/ros2_humble_ws/install/setup.bash' >> ~/.bashrc
```

常用变量：`ROS_DISTRO`、`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`ROS_LOCALHOST_ONLY`、`AMENT_PREFIX_PATH`、`CMAKE_PREFIX_PATH`。`ROS_LOCALHOST_ONLY=1` 会禁止跨主机通信，排查多机问题时尤其要检查。

---

## 4. 常用命令行工具

### 4.1 基础命令

```bash
ros2 --help
ros2 pkg list
ros2 pkg prefix demo_nodes_cpp
ros2 interface list
ros2 interface show geometry_msgs/msg/Twist
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### 4.2 节点和话题

```bash
ros2 node list
ros2 node info /talker
ros2 topic list -t
ros2 topic info /chatter -v
ros2 topic echo /chatter --once
ros2 topic hz /scan
ros2 topic bw /image_raw
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

### 4.3 服务、参数和动作

```bash
ros2 service list -t
ros2 service type /clear
ros2 service call /clear std_srvs/srv/Empty '{}'
ros2 param list /my_node
ros2 param get /my_node use_sim_time
ros2 param set /my_node gain 0.5
ros2 param dump /my_node > my_node.yaml
ros2 action list -t
ros2 action info /navigate_to_pose
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci \
  '{order: 5}' --feedback
```

### 4.4 Launch、Bag 和系统诊断

```bash
ros2 launch my_pkg bringup.launch.py use_sim_time:=true
ros2 bag record -o run_01 /tf /scan /odom
ros2 bag info run_01
ros2 bag play run_01 --clock
ros2 doctor --report
```

录包时应记录 `/tf`、`/tf_static`、传感器、状态和 `/clock`（仿真），并注意磁盘空间和消息类型版本。

---

## 5. 创建功能包

### 5.1 Python 包（ament_python）

```bash
cd ~/ros2_humble_ws/src
ros2 pkg create --build-type ament_python hello_py --dependencies rclpy std_msgs
```

典型结构：

```text
hello_py/
├── hello_py/__init__.py
├── hello_py/talker.py
├── resource/hello_py
├── package.xml
├── setup.py
└── setup.cfg
```

`setup.py` 至少声明入口点：

```python
entry_points={
    'console_scripts': ['talker = hello_py.talker:main'],
}
```

### 5.2 C++ 包（ament_cmake）

```bash
cd ~/ros2_humble_ws/src
ros2 pkg create --build-type ament_cmake hello_cpp --dependencies rclcpp std_msgs
```

`CMakeLists.txt` 的基本构建配置：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

ament_package()
```

`package.xml` 声明元数据和依赖：

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<export><build_type>ament_cmake</build_type></export>
```

构建和运行：

```bash
cd ~/ros2_humble_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hello_cpp hello_py --symlink-install
source install/setup.bash
ros2 run hello_cpp talker
```

---

## 6. 编程接口

### 6.1 C++ 节点、发布与订阅

```cpp
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker"), count_(0) {
    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std_msgs::msg::String message;
      message.data = "hello " + std::to_string(count_++);
      publisher_->publish(message);
    });
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
}
```

订阅者使用 `create_subscription<T>(topic, qos, callback)`；回调可以接收 `const T &`、`T::ConstSharedPtr` 等形式。

### 6.2 Python 节点、发布与订阅

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.count = 0
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        message = String()
        message.data = f'hello {self.count}'
        self.count += 1
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 6.3 Service 服务器和客户端

```python
from example_interfaces.srv import AddTwoInts

self.server = self.create_service(AddTwoInts, 'add_two_ints', self.add)

def add(self, request, response):
    response.sum = request.a + request.b
    return response
```

C++ 使用 `create_service<example_interfaces::srv::AddTwoInts>(...)` 和 `create_client<...>(...)`；客户端应先 `wait_for_service()`，再异步发送请求，并避免在单线程执行器中造成相互等待。

### 6.4 Action 服务器和客户端

Action 需要 `rclcpp_action` 或 `rclpy.action`。服务器通常包含 goal 回调、cancel 回调、accepted 回调，并在后台线程发布 feedback，完成后返回 result。Python 结构示例：

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

self.action_server = ActionServer(
    self, Fibonacci, 'fibonacci', execute_callback=self.execute)

async def execute(self, goal_handle):
    feedback = Fibonacci.Feedback()
    sequence = [0, 1]
    for _ in range(1, goal_handle.request.order):
        sequence.append(sequence[-1] + sequence[-2])
        feedback.sequence = sequence
        goal_handle.publish_feedback(feedback)
    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = sequence
    return result
```

实际项目还应处理取消、异常、超时、资源释放和并发 goal 策略。

### 6.5 参数声明、读取和回调

```python
from rcl_interfaces.msg import SetParametersResult

self.declare_parameter('gain', 1.0)
self.add_on_set_parameters_callback(self.validate_parameters)

def validate_parameters(self, parameters):
    for parameter in parameters:
        if parameter.name == 'gain' and parameter.value <= 0.0:
            return SetParametersResult(successful=False, reason='gain must be positive')
    return SetParametersResult(successful=True)

gain = self.get_parameter('gain').value
```

参数回调应只做快速校验；硬件重配置等耗时操作应设计成异步或显式服务。

---

## 7. 通信机制详解

### 7.1 Topic、Service、Action 选择

| 需求 | 推荐机制 | 原因 |
|---|---|---|
| 连续传感器数据 | Topic | 异步、多订阅者、允许丢帧 |
| 快速查询或触发 | Service | 请求/响应、语义直接 |
| 长时间任务 | Action | 反馈、结果、取消、状态 |
| 配置初始值 | Parameter/YAML | 随节点启动加载 |
| 坐标变换 | TF2 Topic/Buffer | 按时间查询变换链 |

### 7.2 QoS 代码配置

```cpp
rclcpp::QoS qos(rclcpp::KeepLast(10));
qos.reliable().durability_volatile();
auto pub = create_publisher<std_msgs::msg::String>("chatter", qos);
```

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)
self.publisher = self.create_publisher(String, 'chatter', qos)
```

调试时使用 `ros2 topic info -v` 对比发布者和订阅者的 Reliability、Durability、History、Depth，不要只看话题名称。

### 7.3 自定义消息、服务和动作包

接口包通常使用 `ament_cmake`。在 `package.xml` 中声明 `rosidl_default_generators`、`rosidl_default_runtime` 和接口依赖，在 CMake 中：

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Temperature.msg"
  "srv/SetTarget.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
```

接口是跨语言契约；修改字段会影响序列化兼容性、录包回放和下游节点，应通过版本策略管理。

---

## 8. Launch 与参数

### 8.1 Python Launch

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='hello_cpp', executable='talker', name='talker',
            namespace='robot',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('chatter', 'status')],
            output='screen',
        ),
    ])
```

运行时覆盖：

```bash
ros2 launch hello_cpp demo.launch.py use_sim_time:=true
```

### 8.2 YAML 参数文件

```yaml
talker:
  ros__parameters:
    publish_period: 1.0
    use_sim_time: false
```

节点名、命名空间和 YAML 层级必须匹配；也可以使用通配符 `/**` 提供全局参数。Launch 中可同时传入多个参数文件，后者通常覆盖前者。

### 8.3 Composition

Composable Node 可在 `component_container` 或 `component_container_mt` 中动态装载。优点是减少进程和序列化开销、便于组合；代价是崩溃影响面更大、线程和符号调试更复杂。C++ 组件通常使用 `rclcpp_components_register_nodes` 注册，并通过 `ros2 component list/load` 管理。

### 8.4 Lifecycle Launch

Lifecycle 管理器可以按顺序 configure/activate 多个节点。启动文件应明确节点初始状态、转换失败处理、参数文件和 shutdown 行为；传感器未准备好时不要提前激活下游控制节点。

---

## 9. TF2 与坐标变换

### 9.1 坐标系树

TF2（Transform Library 2）维护有时间戳的坐标变换。常见树：

```text
map
└── odom
    └── base_link
        ├── base_footprint
        ├── lidar
        └── camera_link
```

通常 `map -> odom` 由定位发布，`odom -> base_link` 由里程计发布，传感器外参为静态变换。坐标树应保持无环、每个 frame 只有一个父节点。

### 9.2 广播与监听

```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

transform = TransformStamped()
transform.header.stamp = self.get_clock().now().to_msg()
transform.header.frame_id = 'base_link'
transform.child_frame_id = 'lidar'
transform.transform.translation.x = 1.0
self.broadcaster = TransformBroadcaster(self)
self.broadcaster.sendTransform(transform)
```

监听器用 `Buffer.lookup_transform(target, source, time)` 查询；应处理 `LookupException`、`ConnectivityException`、`ExtrapolationException`，并注意查询时间、缓存长度和时间源。

### 9.3 常用工具

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link lidar
ros2 run tf2_ros static_transform_publisher \
  1 0 0 0 0 0 base_link lidar
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

---

## 10. 可视化与调试工具

### 10.1 RViz2

RViz2 通过 Display 订阅 Topic 并使用 TF 将数据转换到 Fixed Frame。显示空白时依次检查 Fixed Frame、TF 链、Topic 类型、QoS、时间戳和 `use_sim_time`。配置可保存为 `.rviz` 文件并在 Launch 中加载。

### 10.2 rqt 与日志

常用插件包括 `rqt_graph`、`rqt_console`、`rqt_plot`、`rqt_topic`：

```bash
rqt_graph
rqt_console
ros2 run rqt_plot rqt_plot /node/value
```

日志级别可在命令行设置：

```bash
ros2 run my_pkg my_node --ros-args --log-level debug
ros2 run my_pkg my_node --ros-args --log-level my_logger:=warn
```

C++ 使用 `RCLCPP_INFO/WARN/ERROR`，Python 使用 `self.get_logger().info()`。不要在高频回调中无条件打印大对象。

### 10.3 rosbag2

```bash
ros2 bag record -a -o experiment_01
ros2 bag play experiment_01 --loop --clock
ros2 bag info experiment_01
```

使用 `--topics` 精确录制；回放仿真数据时让节点使用 `/clock`。录包格式和压缩插件会影响性能、体积及跨发行版兼容性。

### 10.4 ros2 doctor

```bash
ros2 doctor --report
ros2 doctor --report | less
```

它能发现部分安装、环境和网络配置问题，但不能替代 `ros2 topic info -v`、日志和系统级网络检查。

---

## 11. 仿真与导航

### 11.1 Gazebo / Ignition

Gazebo Classic 与新 Gazebo（历史上也称 Ignition）是不同代际的仿真平台。ROS 2 通过插件桥接传感器、关节、时钟和 TF；应根据 Humble 的官方兼容矩阵选择版本。仿真系统的关键接口包括 `/clock`、`sensor_msgs`、`geometry_msgs`、TF 和控制命令。

基本启动思路：

```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 topic echo /clock
```

实际项目还需加载 world、robot description（URDF/Xacro）、控制器、传感器插件和桥接参数。

### 11.2 Nav2

Nav2（Navigation 2）通常包含地图服务器、AMCL 或其他定位、规划器、控制器、行为树导航器、恢复行为和生命周期管理器。常见流程：提供地图和 TF，启动导航栈，通过 `NavigateToPose` Action 发送目标。

```bash
ros2 action list -t | grep navigate
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

排查 Nav2 首先确认 `map -> odom -> base_link`、地图坐标、传感器 QoS、代价地图和生命周期状态。

### 11.3 SLAM

SLAM（Simultaneous Localization and Mapping）同时估计机器人位姿并构建地图。常见输入为激光/深度数据、里程计和 TF，输出地图及 `map -> odom`。建图和定位通常是不同运行模式；地图保存、坐标原点、闭环和传感器外参都会影响结果。

---

## 12. 进阶主题

### 12.1 实时性

ROS 2 的实时系统目标是限制不确定延迟和优先级反转，而不是简单地“使用多线程”。常见措施：实时内核或调度策略、锁定内存、避免回调中动态分配、预分配消息、使用可靠 QoS、隔离高优先级线程、控制日志频率，以及用 tracing/延迟测试验证。DDS 配置、网络拥塞和页面缺失都可能破坏实时性。

### 12.2 安全（SROS2）

SROS2 基于 DDS-Security，通过 keystore、证书和治理文件控制身份认证、加密和访问权限。典型流程：创建 keystore，为 enclave 生成密钥和权限，设置 `ROS_SECURITY_ENABLE`、`ROS_SECURITY_KEYSTORE`、`ROS_SECURITY_STRATEGY`，再启动节点。生产环境必须保护私钥、轮换证书并测试未授权 Topic/Service 的失败行为。

### 12.3 多机通信

两台机器至少需要一致的 `ROS_DOMAIN_ID`、互通的 DDS 网络、正确的 hostname/IP 和允许的 UDP 端口。检查：

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
printenv RMW_IMPLEMENTATION
ping <other-host>
```

跨网段时优先采用 DDS discovery server、VPN 或明确的静态发现配置；不要仅通过修改 ROS_DOMAIN_ID 解决防火墙问题。

### 12.4 Docker 部署

容器中需 source ROS 环境，并根据场景配置网络、设备、GPU、共享内存和 X11/RViz2。ROS 2 通信通常需要 `--net=host` 或等价的 DDS 网络配置；容器内外必须匹配 `ROS_DOMAIN_ID` 和 RMW。镜像中固定 apt 源、rosdep 依赖和 `.repos` 版本，保证可复现构建。

### 12.5 性能调优

先测量再优化。关注端到端延迟、吞吐、CPU、内存、丢包、回调排队和序列化开销。可用 `ros2 topic hz/bw`、系统 profiler、`ros2_tracing`、DDS 统计和 LTTng。常见优化包括合适的 QoS depth、组件组合、进程内通信、零拷贝/loaned message（需 RMW 支持）、消息降采样和合理的 executor 线程数。

---

## 13. 生态与工具链

### 13.1 colcon

`colcon` 按包拓扑排序构建工作区：

```bash
colcon list
colcon build --symlink-install
colcon build --packages-select my_pkg
colcon build --packages-up-to my_pkg
colcon test --packages-select my_pkg
colcon test-result --verbose
```

`--symlink-install` 便于 Python 和资源文件开发；发布或部署时应进行干净构建并验证 install 空间。

### 13.2 rosdep

`rosdep` 把 `package.xml` 中的系统依赖映射到 apt、pip 或其他安装器：

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

依赖 key 找不到时，检查 rosdep 源、ROS_DISTRO、系统发行版以及包名是否只存在于源码工作区。

### 13.3 rosdistro 与包索引

`rosdistro` 描述发行版、仓库版本和依赖规则；包索引可用于查找官方包、API 文档和兼容性。安装包前确认目标发行版是 Humble，避免将 Iron/Jazzy 的文档或二进制混入 Humble 环境。

### 13.4 常用功能包

| 领域 | 常见包 |
|---|---|
| 客户端与接口 | `rclcpp`、`rclpy`、`std_msgs`、`geometry_msgs`、`sensor_msgs` |
| 启动与组件 | `launch`、`launch_ros`、`rclcpp_components` |
| 坐标变换 | `tf2`、`tf2_ros`、`tf2_geometry_msgs` |
| 可视化 | `rviz2`、`rqt_graph`、`rqt_console` |
| 录包 | `rosbag2`、`ros2bag` |
| 导航 | `navigation2`、`nav2_bringup` |
| 仿真 | `gazebo_ros`、对应 Gazebo 插件 |
| 诊断 | `diagnostic_updater`、`ros2doctor` |
| 测试 | `ament_cmake_gtest`、`ament_cmake_pytest`、`launch_testing` |

---

## 14. 常见问题与排查

### 14.1 命令找不到或包不存在

1. 确认已执行 `source /opt/ros/humble/setup.bash`。
2. 确认工作区已执行 `source install/setup.bash`。
3. 检查 `echo $ROS_DISTRO`、`ros2 pkg prefix <package>`。
4. 用 `rosdep install` 补齐依赖，检查包是否真的被 colcon 发现。

### 14.2 Topic 有名称但没有数据

1. `ros2 topic info /name -v` 检查端点和 QoS。
2. `ros2 node info /node` 检查实际解析后的名称。
3. 检查 Publisher 是否真的运行、是否只在收到输入后发布。
4. 对比 `reliable/best_effort` 和 `volatile/transient_local`。
5. 检查 namespace、remap、`ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY`。

### 14.3 RViz2 空白或 TF 报错

1. Fixed Frame 是否存在。
2. `ros2 run tf2_tools view_frames` 检查树是否断裂或成环。
3. 检查消息的 `header.frame_id` 和时间戳。
4. 检查是否同时 source 了仿真时钟和系统时钟，确认 `use_sim_time` 一致。
5. 对传感器显示检查 QoS 是否与驱动匹配。

### 14.4 多机节点互相看不见

检查 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、RMW 实现、防火墙、组播、容器网络和 DDS 配置。先用同一 RMW、同一网段、最小 talker/listener 验证，再逐步加入复杂系统。

### 14.5 服务或 Action 卡住

确认服务器存在且接口类型一致：

```bash
ros2 service list -t
ros2 action list -t
ros2 action info /action_name
```

检查回调是否阻塞、执行器是否只有单线程、Future 是否被同步等待、Action 是否正确处理 goal/cancel/result。

### 14.6 参数设置失败

确认参数已 `declare_parameter`，名称和命名空间正确，YAML 缩进以及 `ros__parameters` 正确。参数回调可能拒绝非法值；用 `ros2 param describe` 检查类型和描述。

### 14.7 colcon 构建失败

清理前先保存日志并定位第一个真正的错误：

```bash
colcon build --event-handlers console_direct+
colcon build --packages-select my_pkg --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

检查 `package.xml`、`CMakeLists.txt`、依赖顺序、接口生成器、头文件安装和环境 source 顺序。不要把后续级联错误误认为根因。

### 14.8 时间、仿真和录包问题

仿真必须由 `/clock` 提供时间，相关节点设置 `use_sim_time=true`。回放时使用 `ros2 bag play --clock`；若 TF 外推失败，检查消息时间戳、录包是否包含 TF、回放速度以及节点启动时机。

### 14.9 开发者的推荐排查顺序

```text
环境 → 进程/节点 → 名称和类型 → Publisher/Subscriber → QoS → 时间 → TF → 算法参数
```

每一步都用 CLI 输出或日志验证，不要仅凭 launch 文件推断运行时状态。对复杂系统，先缩小到一个节点、一个接口和一个最小复现命令，再恢复完整 Launch。

## 结语：从入门到进阶的学习路径

建议按以下顺序练习：先用 demo 节点掌握 Topic、Service、Action、Parameter 和 CLI；再创建一个同时包含 C++ 与 Python 节点的工作区；随后学习 QoS、Launch、TF2、rosbag2 和 lifecycle；最后进入 Nav2、仿真、多机、安全、实时性和性能分析。每个阶段都应完成“写代码、启动、观察图、录包、复现问题、解释 QoS/时间/TF”的闭环。
