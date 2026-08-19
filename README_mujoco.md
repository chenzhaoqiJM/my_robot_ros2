
## 安装环境

ROS2 搭配：

```bash
pip install mujoco
```

## 运行

### 2d 激光雷达底盘

```bash
ros2 launch myrobot_sim_mujoco myrobot_diff_lidar.launch.py enable_viewer:=true
```
### 3d 激光雷达

```bash
ros2 launch myrobot_sim_mujoco myrobot_diff_3d_lidar.launch.py
```

建图导航参考 README 的命令即可