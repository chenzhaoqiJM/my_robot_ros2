
# install deps


```
sudo apt install ros-humble-nav2-graceful-controller
```

# 激光建图导航




# 视觉建图导航


## 视觉建图

启动仿真

```
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd_only.launch.py
```

启动slam

```
ros2 launch myrobot_slam turtlebot3_rgbd.launch.py
```

启动 rviz

```
ros2 launch myrobot_visualization display_rgbd.launch.py
```

## 视觉导航

启动仿真

```
ros2 launch myrobot_sim_gazebo myrobot_diff_rgbd_only.launch.py
```

启动slam

```
ros2 launch myrobot_slam turtlebot3_rgbd.launch.py localization:=true
```

启动 nav2

```
ros2 launch rdk_navigation rgbd_nav2.py
```

启动 rviz

```
ros2 launch myrobot_visualization display_rgbd.launch.py
```