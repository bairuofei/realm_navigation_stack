## Usage
This project contains three modules: 
- Gazebo simulation environment, provided by `p2os_urdf` package in this repo;
- SLAM method. For example, A-LOAM is used. It subscribes `/velodyne_points` published by Gazebo, and performs odometry estimation.
- Path planner. `tare_planner` or `far_planner` is used to serve as high-level path planner.
- Motion planner. `autonomous_exploratin_development_environment` serves as low-level motion planner, performing traversibility analysis, local path generation, and path following.

> NOTE: The world model is provided in `vehicle_simulator/world`. 

```bash
# Start simulation environment
cd /home/ruofei/code/cpp/autonomous_exploration_development_environment
source devel/setup.sh
roslaunch p2os_urdf run.launch

# Start LOAM module
cd /home/ruofei/code/cpp/aloam_ws
source ./devel/setup.bash
roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch 

# Start development environment, for point-to-point navigation
# This module relies on robot state estimation
cd /home/ruofei/code/cpp/autonomous_exploration_development_environment
source ./devel/setup.bash
roslaunch vehicle_simulator system_real_robot.launch

# Launch TARE planner
cd /home/ruofei/code/cpp/tare_planner
source ./devel/setup.sh
roslaunch tare_planner explore_indoor.launch
```

## Problems
1. 导航框架产生的cmd_vel命令不稳定，机器人微调位置时经常左右晃动，最后翻倒。而且，左右晃动影响了terrain traversibility的判断。为缓解此问题，可通过在xacro文件中增大机器人重量，并在`local_planner`中降低机器人的运动速度，但依然存在。
B站说左右晃动是因为控制指令是根据前一个定位消息计算的，有提前量，所以会产生位置的突变？目前的定位频率是5Hz.
2. **SLAM和gazebo使用了两套独立TF系统**。目前，gazebo发布的frame_id为`odom`，而A-LOAM发布的frame_id为`camera_init`。经过`loam_interface` package接收A-LOAM的topic并重命名，A-LOAM发布的frame_id修改为`map`, 位姿frame_id为`sensor`。目前没有直接提供`map`到`odom`的TF变换。可以通过在`loam_interface`中，查询A-LOAM发布的TF消息的时间戳对应的odom消息，并发布相应的变换。
注意，这只是为了保持整个TF系统的完整性，并不改变定位的结果，实际机器人定位以SLAM系统为准。


## Key topics

- "/state_esitmation". Published by vehicleSimulator, using wheel encoder. 如果使用实际机器人，这个topic应当由机器人上运行的SLAM算法，比如LOAM来提供，并通过loam_interface.launch文件，将topic统一重命名为"/state_estimation".



## Functions of the original repo
### Robot state estimation

- The Gazebo in this package only works to provide camere readings and point cloud from LiDAR, while the motion model of the robot is not provided by Gazebo.
Instead, it is provided by the `vehicle_simulator` package through the `/state_estimation` topic. 
The package models the motion dymanics of the robot according to the `cmd_vel` command in a discrete manner.

- In addition, `vehicle_simulator` also publishes TF message between frame `map` and `sensor`.


- `vehicle_simulator`不应该与`loam_interface`同时运行。loamInterface + robot(with SLAM)替代了vehicle_simulator的作用。



### SLAM method interface

The 3rd part SLAM method runs as a node that take "scan" as input, and output the estimated state of the robot.
Therefore, the SLAM node can be used to substitute the `vehicle_simulator` package, which simulates a differentially driven robot.

To use other SLAM methods, change the parameter in the launch file `loam_interface.launch` of `loam_interface` package.


> Currently, the `vehicle_simulator` package has to be used in order to provide ground truth pose for the robot in Gazebo, to generate corresponding pointclounds. 
So nothing changes, but the `/state_estimation`, `/registered_scan` and `tf` should only be published by the SLAM node, rather than the `vehicle_simulator`.


## 基于SLAM的exploration

也可以通过保留`vehicle_simulator`来取代gazebo的运动仿真功能。

需要做什么？
1. 修改vehicleSimulator.cpp文件。gazebo中，机器人的状态依然由vehicle_simulator提供，作为真值。但用于建图的位姿，应当使用从SLAM方法中得到的估计结果。
2. 运行A-LOAM节点，订阅gazebo发布的scan，并输出估计得到的位姿。

1. Waypoint到cmd_vel由vehicleSimulator.cpp发布
2. loamInterface订阅A-LOAM发布的`/aft_mapped_to_init_high_frec`和`/velodyne_cloud_registered`两个topic，并发布map->sensor的tf消息
3. vehicleSimulator根据cmd_vel来进行动力学模拟，并给gazebo赋值。同时发布


因为A-LOAM和AEDE没有重叠的topic，所以可以同时运行A-LOAM节点和AEDE中的system_campus.launch。这样A-LOAM节点只订阅由AEDE发出的`/velodyne_points` topic，自己可以计算机器人的位姿。
而AEDE通过机器人位姿的真值来进行exploration。

如果要把两个系统结合起来，通过A-LOAM来获得机器人的位姿，则运行`system_real_robot.launch`。但注意，我们同样需要在其中include `vehicle_simulator.launch`文件。
因为gazebo系统需要通过vehicle_simulator来获得位姿的真值，因为vehicle_simulator通过订阅`cmd_vel` topic获取了机器人的运动控制指令。
此时，tf消息不需要vehicle_simulator来发布，而是通过loamInterface节点来发布。（实际上是由A-LOAM节点发布的，但是A-LOAM发布的tf和AEDE没有命名冲突。
因此loamInterface通过重命名A-LOAM的frame-id，可以发布与AEDE平台一致的tf消息。）

## 问题


2. 现在机器人无法通过手动指令正常行走。但是gazebo中的机器人也会随着slam的定位结果而抖动。实际上机器人的位置不应该被SLAM而影响。这可能是因为traversability analysis使用的点云，
和SLAM的定位结果有关，因此gazebo中机器人的定位也会受到影响。

3. 机器人和雷达中心重合，导致机器人纯旋转时，获取的点云数据与不旋转时一样，从而slam模块判断机器人没动。这个问题可以通过对雷达与机器人中心添加一个水平方向的offset来解决。
暂时还不能把我自己的robot model直接用来替换TARE中的机器人，因为两个机器人的TF不一样。而且TARE中的机器人本身画得就不是差分驱动的。




## Previous README

<img src="img/header.jpg" alt="Header" width="100%"/>

The repository is meant for leveraging system development and robot deployment for ground-based autonomous navigation and exploration. Containing a variety of simulation environments, autonomous navigation modules such as collision avoidance, terrain traversability analysis, waypoint following, etc, and a set of visualization tools, users can develop autonomous navigation systems and later on port those systems onto real robots for deployment.

Please use instructions on our [project page](https://www.cmu-exploration.com).
