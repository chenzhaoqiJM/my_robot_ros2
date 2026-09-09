# SCAN-Planner 接入 myrobot Gazebo 仿真指南

> 本文是原二维稳定模式。完整保留 FAST-LIO 三维位姿的新模式见
> `SCAN_PLANNER_GAZEBO_3D_GUIDE_CN.md`；两者互不覆盖。

## 1. 已验证环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic（`gazebo_ros`）
- SCAN-Planner `ros2-community` 分支，提交 `d0b921c`
- 仿真机器人：`myrobot_diff_3d_lidar`

本工作区已经加入专用适配层。它将 Gazebo 的 `/odom`、`/points` 转换为
SCAN-Planner 使用的车体位姿、雷达位姿和局部点云，并将规划器输出接到
Gazebo 的 `/cmd_vel`。

## 2. 数据流

```text
Gazebo /odom ──────> scan_planner_adapter ──> /scan_planner/body_pose
                                      └─────> /scan_planner/sensor_pose
Gazebo /points ────> 地面回波过滤 ──────────> /scan_planner/cloud
SCAN-Planner ──────> /planning/bspline ─────> closed_loop_controller
closed_loop_controller ─────────────────────> /cmd_vel ──> Gazebo
```

适配参数按照当前 URDF 设置：雷达相对 `base_footprint` 的偏移为
`(0.07, 0.0, 0.144) m`，规划车体高度为 `0.25 m`。差速底盘不能横移，
因此闭环控制器的 `max_vy` 固定为 0。

## 3. 获取和构建

SCAN-Planner 已位于工作区：

```bash
cd /media/chenzhaoqi/data/tmp/myrobot_ws/src/SCAN-Planner
git switch ros2-community
```

从工作区根目录构建 CPU 版本：

```bash
cd /media/chenzhaoqi/data/tmp/myrobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to scan_planner myrobot_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

当前机器没有 `libglm-dev`，但 CPU 感知后端并不使用 GLM，仓库内的 CMake
已修正为仅在 `USE_GPU=ON` 时查找它。若要开启 GPU 后端，需先安装：

```bash
sudo apt install libglm-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

## 4. 一键启动

```bash
cd /media/chenzhaoqi/data/tmp/myrobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch myrobot_navigation scan_planner_gazebo.launch.py
```

该命令默认同时启动 Gazebo、SCAN-Planner、差速闭环控制器和 RViz2。
在无桌面的终端中可运行：

```bash
ros2 launch myrobot_navigation scan_planner_gazebo.launch.py \
  gui:=false start_rviz:=false
```

若 Gazebo 已经启动，只启动规划部分：

```bash
ros2 launch myrobot_navigation scan_planner_gazebo.launch.py \
  start_gazebo:=false
```

## 5. 下发目标

在 RViz2 中确认 Fixed Frame 为 `world`，使用 **2D Goal Pose** 点击目标。
也可以在新终端直接发送目标：

```bash
source /media/chenzhaoqi/data/tmp/myrobot_ws/install/setup.bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 2.0, y: 0.0, z: 0.25}, orientation: {w: 1.0}}}"
```

当前 `myworld1` 中，`(2.0, 0.0)` 是直线路径测试；`(6.5, 0.0)` 会穿过
大砖墙所在方向，可观察局部重规划和绕障。不要把目标设为 `(6.0, 0.0)`：
该点距离砖墙边缘过近，会被膨胀占据栅格判为不可达。

## 6. 运行检查

```bash
# 输入频率：车体位姿约 29 Hz，过滤点云约 10 Hz
ros2 topic hz /scan_planner/body_pose
ros2 topic hz /scan_planner/cloud

# 规划器是否生成 B 样条
ros2 topic echo /planning/bspline --once

# 控制输出和最终位置
ros2 topic echo /cmd_vel
ros2 topic echo /odom --once

# 查看局部膨胀占据点云
ros2 topic echo /grid_map/occupancy_inflate --once --field width
```

正常日志会依次出现 `GEN_NEW_TRAJ`、`EXEC_TRAJ`、`Received trajectory`，
到达后回到 `WAIT_TARGET`，同时 `/cmd_vel` 归零。

## 7. 本机实测结果

- 目标 `(2.0, 0.0)`：规划成功，最终里程计约 `(2.080, 0.000)`。
- 目标 `(6.5, 0.0)`：检测正前方障碍并向负 y 方向绕行，最大侧向绕行约
  `y=-0.82 m`，最终约 `(6.452, -0.028)`。
- 到达后 `/cmd_vel` 的线速度和角速度均为 0。
- `/grid_map/occupancy_inflate` 能持续发布，绕障测试结束时约 2.58 万点。

## 8. 调参位置

集成参数位于：

```text
my_robot_ros2/myrobot_navigation/launch/scan_planner_gazebo.launch.py
```

常用参数：

- `grid_map.double_cylinder_radius`：当前 0.16 m，表示车体半径和安全余量。
- `min_sensor_z`：当前 -0.10 m，过滤雷达坐标系下的地面回波。
- `manager.max_vel` / `optimization.max_vel` / `max_vx`：当前 0.35 m/s。
- `fsm.planning_horizon`：当前 3.5 m。
- `fsm.max_replan_fail_count`：当前 50，目标不可达时限制连续失败重试。

若更换机器人尺寸、雷达安装位置或 Gazebo 世界，应优先重新标定这些参数。

## 9. 注意事项

SCAN-Planner 原本针对可全向运动并能跨越楼层的 Go2 四足机器人。本适配只使用
其三维占据建图、碰撞检测、B 样条优化和局部避障能力；差速底盘不执行横移和
爬楼动作。对于狭窄场景，规划出的连续曲线可能比差速车实际可跟踪的曲率更激进，
应降低速度并增大车体膨胀半径。
