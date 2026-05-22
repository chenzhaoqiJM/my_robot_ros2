#!/usr/bin/env python3

import math
import sys
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult



def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q



def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation = quaternion_from_yaw(yaw)
    return pose



def build_square_waypoints(navigator: BasicNavigator, side_length: float = 1.0) -> List[PoseStamped]:
    return [
        make_pose(navigator, side_length, 0.0, 0.0),
        make_pose(navigator, side_length, -side_length, -math.pi / 2.0),
        make_pose(navigator, 0.0, -side_length, -math.pi),
        make_pose(navigator, 0.0, 0.0, math.pi / 2.0),
    ]



def main() -> int:
    rclpy.init()
    navigator = BasicNavigator()

    try:
        # navigator.waitUntilNav2Active()

        waypoints = build_square_waypoints(navigator, side_length=1.0)
        navigator.followWaypoints(waypoints)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and hasattr(feedback, 'current_waypoint'):
                print(f'正在前往第 {feedback.current_waypoint + 1} / {len(waypoints)} 个点')
            rclpy.spin_once(navigator, timeout_sec=0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('正方形四点巡航完成')
            return 0
        if result == TaskResult.CANCELED:
            print('任务已取消')
            return 1

        print('任务失败')
        return 2
    except KeyboardInterrupt:
        navigator.cancelTask()
        print('收到中断，已取消任务')
        return 130
    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
