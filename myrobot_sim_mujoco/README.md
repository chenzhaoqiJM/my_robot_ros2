# myrobot_sim_mujoco

MuJoCo based differential-drive simulation that matches the existing myrobot ROS 2 interfaces.

## Interfaces

Published topics:

- `/clock` (`rosgraph_msgs/Clock`)
- `/odom` (`nav_msgs/Odometry`)
- `/scan` (`sensor_msgs/LaserScan`)
- `/imu/data` (`sensor_msgs/Imu`)
- `/tf`: `odom -> base_footprint`

The launch file also starts `robot_state_publisher` with the existing `myrobot_sim_gazebo/xacro/myrobot_lidar.xacro`, so the internal frames stay aligned with the current Gazebo model:

- `base_footprint -> base_link`
- `base_link -> laser_link`
- `base_link -> imu_link`

## Dependency

Install the MuJoCo Python package in the ROS 2 Python environment:

```bash
pip install mujoco
```

## Launch

```bash
ros2 launch myrobot_sim_mujoco myrobot_diff_lidar.launch.py
```

For SLAM:

```bash
ros2 launch myrobot_navigation nav2_for_slam.launch.py
```

For navigation with an existing map:

```bash
ros2 launch myrobot_navigation lidar_nav2.launch.py
```

## RGB-D camera

The RGB-D model matches `myrobot_rgbd2.xacro`: 640 x 480, 60 degree
horizontal field of view, 0.02-8.0 m depth range, 20 Hz publication rate,
and 0.007 m Gaussian depth noise. The MuJoCo bridge publishes all TF frames
for this variant, so no `robot_state_publisher` is started.

```bash
ros2 launch myrobot_sim_mujoco myrobot_rgbd.launch.py enable_viewer:=true
```
