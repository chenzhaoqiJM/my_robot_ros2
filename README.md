# 编译命令

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```


# install deps


```
sudo apt install ros-humble-nav2-graceful-controller
```

# 激光建图导航


启动仿真环境，或者使用mujoco仿真环境

```bash
ros2 launch myrobot_sim_mujoco myrobot_diff_lidar.launch.py
```

## 建图

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

## 导航

```bash
ros2 launch myrobot_navigation lidar_nav2.launch.py
```

### 导航可视化

```bash
ros2 launch myrobot_visualization display_navigation.launch.py
```

# 视觉建图导航


## 视觉建图

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd_only.launch.py
```

启动slam

```bash
ros2 launch myrobot_slam turtlebot3_rgbd.launch.py
```

启动 rviz

```bash
ros2 launch myrobot_visualization display_rgbd.launch.py
```

## 视觉导航

启动仿真

```bash
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd_only.launch.py
```

启动slam

```bash
ros2 launch myrobot_slam turtlebot3_rgbd.launch.py localization:=true
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

## 3d 激光雷达导航

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