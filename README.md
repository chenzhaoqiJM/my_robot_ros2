
# 激光建图导航




# 视觉建图导航


## 视觉建图

启动仿真

```
ros2 launch lekiwi_sim_gazebo turtlebot3_waffle_world.launch.py
```

启动slam

```
ros2 launch rdk_localization turtlebot3_rgbd.launch.py
```

启动 rviz

```
ros2 launch rdk_visualization display_rgbd.launch.py
```

## 视觉导航

启动仿真

```
ros2 launch lekiwi_sim_gazebo turtlebot3_waffle_world.launch.py
```

启动slam

```
ros2 launch rdk_localization turtlebot3_rgbd.launch.py localization:=true
```

启动 nav2

```
ros2 launch rdk_navigation rgbd_nav2.py
```

启动 rviz

```
ros2 launch rdk_visualization display_rgbd.launch.py
```