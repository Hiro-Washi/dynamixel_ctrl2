#!/usr/vin/env python3
#-*- coding: utf-8 -*-
#
#

import numpy
import math
import threading
import time

import rclpy
from rclpy.node import Node
#import rosparam # !!!!
from std_msgs.msg import Bool, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand
# -- Custom Message --
from happymimi_msgs2.srv import StrTrg
from happymimi_recognition_msgs2.srv import PositionEstimator
from mimi_mani_msgs2.srv import ArmControl # !!!!

# ÉmÅ[Éhê∂ê¨ÇÕäOïî
class MotorController(Node):
  def __init__(self):
    self.create_service(DynamixelStateList, 
                        '/dynamixel_workbench/dynamixel_state',
                        self.getMotorStateCB)
    self.motor_pub = self.create_publisher(Float64MultiArray,
                                           'dynamixel_workbench/joint_trajectory',
                                           queue_size = 10)
    self.motor_angle_pub = self.create_publisher(Float64MultiArray,
                                                 '/servo/angle_list',
                                                 queue_size = 10)
    # ServiceClient
    self.motor_client = self.create_client(DynamixelCommand,
                                         '/dynamixel_workbench/dynamixel_command')
    while not self.motor_client.wait_for_service(1.0):
      self.get_logger().warn('waiting for service... /dynamixel_workbench/dynamixel_command')
    dyna_cmd_future = self.motor_client.call_async(request)
    # remove node from executor added by rclpy.spin().
    rclpy.spin_until_future_complete(self, dyna_cmd_future)
    

  def getMotorStateCB(self, state):
    
  def motorAnglePub(self, event):

  def degToStep(self, deg):
  
  def stepToDeg(self, step):
  
  def motorPub(self, joint_name, joint_angle, execute_time = 0.8):
  
  def setPosition(self, motor_id, position_value):
  
  def setCurrent(self, motor_id, current_value):
    
class JointController(MotorController):
  def __init__(self):

if __name == '__main__':
  rclpy.init(args=args)  # initialize node
  node = Node("motor_controller2")
  
  experiment = ManipulateArm()
  rclpy.spin(node)
