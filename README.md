# MR_ros_interface

一个基于ROS的多机器人探索与规划系统ROS接口，并不提供集中式规划算法。支持多台机器人的协同信息收集以及动作执行，完成多机器人环境探索任务。仅需要修改对应的规划算法接口，即可同步收集机器人的视觉、深度和位姿数据，进行自主规划，并向各机器人发送运动指令。

![样例3个机器人的图.svg](样例3个机器人的图.svg)

## 功能特点

- 支持2+台机器人协同探索（可扩展）
- 实时同步收集RGB图像、深度图像和位姿数据，并进行滤波修正
- 动态参数配置与机器人使能控制
- 支持RViz可视化监控
- 提供多种调试工具（检查话题、TF变换、深度数据）
- 支持手动控制与自动规划两种模式

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- CMake 3.10+
- 依赖包：
  - `rospy`
  - `tf2_ros`
  - `cv_bridge`
  - `sensor_msgs`
  - `geometry_msgs`
  - `numpy`
  - `opencv-python`
  - `dynamic_reconfigure`

## 安装步骤

1. 克隆仓库到ROS工作空间的`src`目录下：

```bash
cd ~/catkin_ws/src
git clone <本仓库地址>
cd ..
```

2. 编译工作空间：

```bash
catkin_make
source devel/setup.bash
```

## 快速开始

以2台机器人的场景为例：

1. 启动主程序：

```bash
roslaunch explore2 2r_pet_mr.launch
```

2. 在新终端中启动自动规划请求节点：

```bash
rosrun request_cli auto_planning
```

3. 等待算法准备成功，在auto_planning的终端处按下 Y 或者 y .
系统将自动开始多机器人协同探索。RViz会自动启动以可视化机器人状态和传感器数据。

## 使用说明

### 启动不同场景

系统提供了多种预设场景的launch文件：

- 2台机器人（petbot + mr600）：`2r_pet_mr.launch`
```bash
roslaunch explore2 2r_pet_mr.launch
```

- 2台机器人（petbot + spark）：`2r_pet_spa.launch`
```bash
roslaunch explore2 2r_pet_spa.launch
```

- 3台机器人：`3r.launch`
```bash
roslaunch explore2 3r.launch
```

- 自定义场景：`s2.launch`
```bash
roslaunch explore2 s2.launch
```

### 发送规划请求

有多种方式可以触发规划请求：

1. **自动规划**（推荐）：
```bash
rosrun request_cli auto_planning
```

2. **快速规划**：
```bash
rosrun request_cli fast_planning
```

3. **键盘手动触发**：
```bash
rosrun request_cli keyboard_planning_request
# 按下 'Y' 键发送规划请求，'q' 键退出
```

### 手动控制机器人

系统提供了针对不同机器人的手动控制脚本：

- Petbot:
```bash
rosrun explore2 petbot_teleop_key.py
```

- Robuster:
```bash
rosrun explore2 robuster_teleop_key.py
```

- Spark:
```bash
rosrun explore2 spark_teleop_key.py
```

## 核心组件

### 1. 主规划节点 (`mr_planning_2r_dyn.py`)

系统核心节点，负责：
- 接收规划请求
- 收集所有机器人的数据
- 执行多机器人规划算法
- 向各机器人发送运动指令
- 支持动态参数配置

### 2. 数据收集节点 (`message_collector_syn.py`)

每个机器人对应一个数据收集节点，负责：
- 同步采集RGB图像、深度图像和位姿数据
- 对传感器数据进行滤波处理
- 通过服务(`DataCollection.srv`)提供数据查询接口

### 3. 请求客户端 (`request_cli`)

提供多种规划请求方式：
- `auto_planning.cpp`: 自动循环发送规划请求
- `fast_planning.cpp`: 快速连续发送规划请求
- `keyboard_planning_request.cpp`: 键盘手动触发规划

### 4. 辅助工具

- `check_depth.py`: 检查深度图像数据
- `check_tf.py`: 检查TF变换关系
- `check_topic.py`: 检查ROS话题是否正常发布

## 参数配置

主要参数可在launch文件中配置：

```xml
<!-- 全局参数配置 -->
<arg name="num_agents" default="2" />  <!-- 机器人数量 -->
<arg name="debug_mode" default="False"/>  <!-- 调试模式 -->
<rosparam param="dx">[0.0, 0]</rosparam>  <!-- X方向偏移量 -->
<rosparam param="dy">[0.0, -24]</rosparam>  <!-- Y方向偏移量 -->
<rosparam param="cmd_vel_topic_table">["/robot1/cmd_vel", '/robot2/cmd_vel','/robot3/cmd_vel']</rosparam>  <!-- 速度控制话题 -->
```

每个机器人的配置：

```xml
<!-- 机器人1配置 -->
<arg name="robot1_rgb_topic" default="/robot1/camera/color/image_raw/compressed" />
<arg name="robot1_depth_topic" default='/robot1/camera/aligned_depth_to_color/image_raw/compressedDepth' />
<arg name="robot1_tf_source" default="robot1/base_link" />
<arg name="robot1_tf_target" default="robot1/map" />
<arg name="robot1_window_size_x" default="20" />  <!-- X方向滤波窗口大小 -->
<arg name="robot1_window_size_y" default="20" />  <!-- Y方向滤波窗口大小 -->
<arg name="robot1_window_size_yaw" default="20" />  <!-- 旋转角滤波窗口大小 -->
```

动态参数配置可通过`rqt_reconfigure`工具进行实时调整：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## 文件结构

```
├── CMakeLists.txt
├── LICENSE
├── README.md
├── explore2
│   ├── CMakeLists.txt
│   ├── cfg                  # 动态配置文件
│   │   └── MRPlanner.cfg
│   ├── include              # 头文件
│   ├── launch               # 启动文件
│   ├── package.xml
│   ├── rviz_config          # RViz配置文件
│   ├── scripts              # Python脚本
│   │   ├── message_collector_syn.py  # 数据收集节点
│   │   ├── mr_planning_2r_dyn.py     # 主规划节点
│   │   └── ...              # 其他脚本
│   ├── src                  # C++源代码
│   ├── srv                  # 服务定义
│   │   └── DataCollection.srv
│   └── tools                # 辅助工具
│       ├── check_depth.py
│       ├── check_tf.py
│       └── check_topic.py
├── request_cli              # 请求客户端
│   ├── ...
└── 样例3个机器人的图.svg
```

## 环境要求与依赖

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- 必要的ROS包：`roscpp`, `rospy`, `tf2`, `tf2_ros`, `cv_bridge`, `sensor_msgs`, `geometry_msgs`
- Python库：`numpy`, `opencv-python`, `icecream`

## 许可证

[LICENSE](LICENSE)

## 注意事项

1. 确保在启动本系统前，各机器人的驱动程序已正确启动
2. 首次运行时，可能需要等待各节点初始化完成
3. 如遇到数据同步问题，可尝试调整launch文件中的滤波窗口大小参数
4. 确保各机器人的TF变换关系正确发布

如果在使用过程中遇到问题，请提交issue或联系开发者。