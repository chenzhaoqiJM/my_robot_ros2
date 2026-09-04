# 使用指南

## 编译命令

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## 安装依赖

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-controller \
  ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-graceful-controller \
  python3-zmq python3-serial python3-transforms3d \
libspdlog-dev libconsole-bridge-dev liborocos-kdl-dev nlohmann-json3-dev liblgpio-dev \
liblttng-ust-dev libgpiod-dev ros-humble-camera-info-manager ros-humble-slam-toolbox \
ros-humble-cartographer ros-humble-cartographer-ros ros-humble-nav2*
```

## 2d激光建图导航仿真环境

启动仿真环境，或者使用mujoco仿真环境

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_lidar.launch.py
```

## 2d lidar 建图

### slam-toolbox sync

```bash
ros2 launch myrobot_slam online_sync_sim.launch.py
```

### slam-toolbox async

```bash
ros2 launch myrobot_slam online_async_sim.launch.py
```

### cartographer

```bash
ros2 launch myrobot_slam cartographer_sim.launch.py
```

### 建图可视化

```bash
ros2 launch myrobot_visualization display_slam.launch.py
```

### 保存地图

```bash
ros2 run nav2_map_server map_saver_cli -f my_map2
```

## 2d lidar 导航

```bash
ros2 launch myrobot_navigation lidar_nav2.launch.py
```

### 导航可视化

```bash
ros2 launch myrobot_visualization display_navigation.launch.py
```

## 2d lidar 无 odom 建图导航

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd_no_odom.launch.py

ros2 launch myrobot_slam cartographer_odom.launch.py

ros2 launch myrobot_navigation nav2_for_slam.launch.py

ros2 launch myrobot_visualization display_navigation.launch.py
```

## 视觉建图（RTAB）

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd.launch.py
```

启动slam

```bash
ros2 launch myrobot_slam rtabmap_rgbd.launch.py
```

启动 rviz

```bash
ros2 launch myrobot_visualization display_rgbd.launch.py
```

### no odom 模式

使用 rtab icp

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd_no_odom.launch.py


ros2 launch myrobot_slam rtabmap_rgbd_no_odom.launch.py
```

使用二维激光雷达里程计

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd_no_odom.launch.py

ros2 launch myrobot_slam cartographer_odom.launch.py

ros2 launch myrobot_slam rtabmap_rgbd.launch.py
```

## 视觉导航（RTAB）

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd.launch.py
```

启动slam

```bash
ros2 launch myrobot_slam rtabmap_rgbd.launch.py localization:=true
```

启动 nav2

```bash
ros2 launch myrobot_navigation rgbd_nav2.py
```

启动 rviz

```bash
ros2 launch myrobot_visualization display_rgbd.launch.py
```

## 3D 激光雷达建图（RTAB）

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_3d_lidar.launch.py
```

启动slam

slam 里面包含了实验性的激光雷达里程计

```bash
ros2 launch myrobot_slam rtabmap_3d_lidar.launch.py
```

启动可视化

```bash
ros2 launch myrobot_visualization display_rtab_3dlidar.launch.py
```

## 3d 激光雷达导航（RTAB）

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_3d_lidar.launch.py
```

启动 slam 定位模式

```bash
ros2 launch myrobot_slam rtabmap_3d_lidar.launch.py localization:=true
```

启动导航

```bash
ros2 launch myrobot_navigation lidar_3d_rtabmap_nav2.py
```

可视化

```bash
ros2 launch myrobot_visualization display_rgbd.launch.py
```

## 3d 激光雷达建图（fastlio）

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_3d_lidar_lio.launch.py
```

启动 slam

```bash
ros2 launch myrobot_slam fast_lio_sim.launch.py
```

启动可视化

```bash
ros2 launch myrobot_visualization display_lio.launch.py
```

## 3d 激光雷达导航（fastlio）

### 纯本地 odom 导航

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_3d_lidar_lio.launch.py
```

启动 slam

```bash
ros2 launch myrobot_slam fast_lio_sim.launch.py
```

启动 local 导航

```bash
ros2 launch myrobot_navigation lio_nav2.launch.py
```

### FAST-LIO里程计 + AMCL

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_3d_lidar_lio.launch.py
```

启动 slam

```bash
ros2 launch myrobot_slam fast_lio_sim.launch.py
```

启动转换节点

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/points \
  -r scan:=/scan \
  -p target_frame:=base_footprint \
  -p min_height:=0.02 \
  -p max_height:=0.35 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p range_min:=0.10 \
  -p range_max:=12.0 \
  -p use_inf:=true
```

启动 amcl 导航

```bash
ros2 launch myrobot_navigation lidar_nav2.launch.py
```

### FAST-LIO里程计绑定map

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_3d_lidar_lio.launch.py
```

启动 slam

```bash
ros2 launch myrobot_slam fast_lio_sim.launch.py
```

启动导航

```bash
ros2 launch myrobot_navigation lio_map_nav2.launch.py
```

### FAST-LIO里程计+2d slam建图 + nav2

可以提供动态建图能力

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_3d_lidar_lio.launch.py
```

启动 slam

```bash
ros2 launch myrobot_slam fast_lio_sim.launch.py
```

启动导航

```bash
ros2 launch myrobot_navigation lio_slam_nav2.launch.py
```
