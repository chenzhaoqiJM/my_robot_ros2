# SCAN-Planner + FAST-LIO 三维仿真指南

这套模式是新增链路，不修改也不替换原有二维启动文件
`scan_planner_gazebo.launch.py`、`fast_lio_sim.launch.py` 和
`fast_lio_odom_relay.py`。

## 1. 能力边界

- FAST-LIO 输出完整 `x/y/z + roll/pitch/yaw`，不再把 z 清零。
- 三轴线速度和三轴角速度由连续位姿估算，并写入 `/odom_3d`。
- 雷达位姿使用完整四元数和三维外参计算。
- 点云不裁剪 z，坡道、台阶、楼层和上下方障碍都会保留给 SCAN-Planner。
- 当前 Gazebo 机器人仍是差速底盘，只能执行 `vx + yaw`，不能真实执行
  `vy/vz/roll/pitch`。要完成飞行或爬楼动作，需换成具有相应自由度的机器人和控制器。

## 2. 数据流

```text
/points + /imu/data_raw -> FAST-LIO -> /Odometry
                                      -> fast_lio_odom_relay_3d -> /odom_3d
/odom_3d + /points -> scan_planner_adapter_3d
                   -> /scan_planner_3d/body_pose
                   -> /scan_planner_3d/sensor_pose
                   -> /scan_planner_3d/cloud
SCAN-Planner -> /planning/bspline -> 平面闭环控制器 -> /cmd_vel
```

`/odom_3d.twist.linear` 是世界坐标系速度，与 SCAN-Planner 当前源码的读取方式一致。

## 3. 构建

```bash
cd /media/chenzhaoqi/data/tmp/myrobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to scan_planner myrobot_slam myrobot_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 4. 一键启动

不要同时启动旧二维模式。直接运行：

```bash
ros2 launch myrobot_navigation scan_planner_gazebo_3d.launch.py
```

不启动 RViz：

```bash
ros2 launch myrobot_navigation scan_planner_gazebo_3d.launch.py \
  start_rviz:=false
```

如果 Gazebo 和 FAST-LIO 已分别启动：

```bash
ros2 launch myrobot_navigation scan_planner_gazebo_3d.launch.py \
  start_gazebo:=false start_fast_lio:=false
```

此时外部 FAST-LIO 必须提供 `/Odometry`，并另行启动新转发器：

```bash
ros2 run myrobot_slam fast_lio_odom_relay_3d --ros-args -p use_sim_time:=true
```

## 5. 检查完整三维数据

```bash
ros2 topic hz /Odometry
ros2 topic hz /odom_3d
ros2 topic hz /scan_planner_3d/cloud
ros2 topic echo /odom_3d --once
```

重点检查 `/odom_3d`：

- `pose.pose.position.z` 未被强制设为 0；
- `pose.pose.orientation.x/y/z/w` 是完整四元数；
- 运动时 `twist.twist.linear.z` 和 `angular.x/y/z` 可随估计变化。

## 6. 下发地面目标

当前差速车应将目标 z 设为 FAST-LIO 初始机体高度，即约 0：

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

不要给当前差速车下发不同楼层的 z 目标：规划器可能生成三维轨迹，但底盘控制器
无法执行 z 方向速度。将来换成无人机或腿式机器人时，可保留 FAST-LIO、适配器和
SCAN-Planner，只需替换末端控制器，并开放对应自由度。

本机实测目标 `(2.0, 0.0)` 时，规划器成功生成并重规划 3 条 B 样条，最终
`/odom_3d` 约为 `(2.066, 0.003, -0.053)`，到达后 `/cmd_vel` 全部归零。
静止与行驶期间观察到的 z 小幅变化来自 FAST-LIO 三维估计，不会被适配层清零。

## 7. 与原二维模式的关系

| 模式 | 里程计 | z/姿态 | 点云处理 | 启动文件 |
|---|---|---|---|---|
| 原二维稳定版 | Gazebo `/odom` | 固定高度、主要使用 yaw | 过滤地面 | `scan_planner_gazebo.launch.py` |
| 新三维版 | FAST-LIO `/odom_3d` | 完整 6-DoF | 不裁剪 z | `scan_planner_gazebo_3d.launch.py` |

需要回退时直接运行原二维启动命令即可，无需恢复任何文件。
