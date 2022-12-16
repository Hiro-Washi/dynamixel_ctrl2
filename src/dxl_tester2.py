#!/usr/bin/env python3
# -*- config: utf-8 -*-

import math
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rclpy.init()
node = rclpy.create_node('dxl_tester2')

def degToRad(deg):
    return math.radians(deg)

def motorPub(joint_name, joint_angle, execute_time=1.0):
  rate = node.create_rate(0.5)
    # Publisherの定義
  motor_pub = node.create_publisher(JointTrajectory,
                                    '/dynamixel_workbench/joint_trajectory', 
                                    queue_size=10)
  rate.sleep(0.5)
  
  # publishするデータの定義
  msg = JointTrajectory()
  msg.header.stamp = node.Time.now()
  msg.joint_names = joint_name
  msg.points = [JointTrajectoryPoint()]
  msg.points[0].positions = list(map(degToRad, joint_angle))
  msg.points[0].time_from_start = node.Time(execute_time)
  # publish
  motor_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    motorPub(['pan'], [30])