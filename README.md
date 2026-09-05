# 扫荡北航先锋-ROS端

## 独立 Conda 测试环境

```bash
conda env create -f environment.yml
conda activate ros-buaa-noetic
catkin_make -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" -j2
source devel/setup.bash
pytest -q
roscore
```

另一个激活相同环境、加载 `devel/setup.bash` 的终端可运行：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090
```

此环境使用 RoboStack Noetic，支持在 Debian 上编译本仓库的消息、服务和运行节点。
`empy<4` 是 ROS 1 C++ 消息生成器的兼容性约束。
`PYTHONNOUSERSITE=1` 避免用户目录中其它项目的 Python 依赖进入测试环境。
真实机器人启动文件仍依赖 WPB_HOME、激光雷达和 Kinect 驱动及硬件。

本项目是北京航空航天大学软件工程小组作业的ROS端代码仓库，用于实现机器人的自主建图、导航、数据采集等功能，并能通过`rosbridge`与云端后端进行通信，接收指令并上报数据。

## 1. ROS端功能介绍

本ROS端系统基于ROS (Robot Operating System) 构建，核心功能包为 `Aft_g1`。系统整体采用状态机模式进行管理，主要包括 **空闲(IDLE)**、**建图(MAPPING)** 和 **导航(NAVIGATION)** 三种状态。

### 主要功能节点

系统通过`real_robot_map_build_and_nav.launch`启动文件进行统一启动，包含了以下核心功能：

-   硬件驱动：
    -   `WPB_HOME` 机器人底盘驱动。
    -   `Rplidar` 激光雷达驱动。
    -   `Kinect2` 深度相机驱动。
-   SLAM与地图：
    -   使用`gmapping`包进行实时二维环境地图构建（SLAM）。
    -   使用`map_server`加载、保存和提供地图服务。
-   自主导航：
    -   使用`move_base`功能包进行路径规划和自主导航。
    -   集成了自定义的局部路径规划器`wpbh_local_planner`以适应特定场景。
-   数据采集与监控：
    -   `master_node`节点会订阅Kinect2的图像话题，并将图像与机器人当前位姿（通过`amcl`获取）进行关联，保存在本地`/tmp/img`目录下。这为后端提供了带有空间位置信息的数据源。
    -   通过`RViz`对机器人状态、传感器数据、地图和导航过程进行可视化。

## 2. 与后端/前端的通信

ROS端通过`rosbridge`（WebSocket协议）与外部系统（如Web后端）进行双向通信，构成了整个云-边-端架构的关键一环。

-   后端 -> ROS (指令下发)
    -   后端通过WebSocket连接到`rosbridge`，以JSON格式调用ROS Service。
    -   核心服务为`Aft_g1/MasterNode`，服务名称为`/master_node_service`。后端通过调用此服务，可以向ROS端发送高级指令，如开始建图、开始导航、指定巡航点等。
    -   `master_node.py`节点作为指令入口，负责解析来自后端的请求，并将其转换为ROS内部的话题消息，分发给相应的功能节点。

-   ROS -> 后端 (数据上报)
    -   `master_node.py`中集成了HTTP客户端（`requests`库），可以将采集到的数据（如带有位置信息的图片）主动上报给后端服务器。

## 3. 主控节点通信

`Aft_g1`包内的所有自定义节点构成了一个分工明确的控制系统，其核心是`master_node.py`。

-   `master_node.py` (主控节点)
    -   角色：作为系统的"大脑"，维护系统状态机，接收外部指令，并协调其他节点的工作。
    -   服务：提供`/master_node_service` (类型: `Aft_g1/MasterNode`)服务，作为与后端通信的唯一入口。
    -   发布话题：
        -   `/mapping_cmd` (`Aft_g1/MappingCmd`): 向`mapping_control.py`发布建图相关指令（开始、结束、保存等）。
        -   `/nav_control` (`Aft_g1/NavControl`): 向`nav_control_node.py`发布导航相关指令（加载地图、设置目标点、开始巡航等）。
    -   订阅话题：
        -   `/amcl_pose`: 获取机器人在地图中的精确位姿。
        -   `/kinect2/hd/image_color_rect`: 接收摄像头图像数据用于保存。

-   其他关键节点
    -   `mapping_control.py`: 负责执行具体的建图逻辑。
    -   `nav_control_node.py`: 负责执行具体的导航逻辑，如调用`move_base`服务。
    -   其他辅助节点：如`pose_transf.py`（坐标变换）、`warning_sound.py`（声音警报）等，共同完成了整个系统的功能闭环。

## 4. 运行方式

### a. 编译工作空间

在第一次运行前或修改了C++/Python代码后，需要对`catkin`工作空间进行编译。在项目根目录下执行：

```bash
catkin_make
```

编译成功后，需要刷新环境变量，使ROS能够找到新生成的包和可执行文件：

```bash
source devel/setup.bash
```
建议将此命令添加到`~/.bashrc`中，以避免每次打开新终端都需要手动执行。

### b. 启动系统

运行本系统需要启动两个核心进程，建议在两个独立的终端中分别执行。

1.  启动`rosbridge`服务

    这个服务是ROS与外部世界通信的桥梁。
    ```shell
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

2.  启动机器人主程序

    这个launch文件会启动所有硬件驱动、核心算法节点和`Aft_g1`包的全部节点。
    ```shell
    roslaunch Aft_g1 real_robot_map_build_and_nav.launch
    ```

系统启动后，即可通过后端服务向机器人下发指令。
