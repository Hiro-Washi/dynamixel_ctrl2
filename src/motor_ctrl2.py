#!/usr/vin/env python3
#-*- coding: utf-8 -*-

import numpy
import math
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand
# -- Custom Message --
from happymimi_msgs2.srv import StrTrg
from happymimi_recognition_msgs2.srv import PositionEstimator
from mimi_mani_msgs2.srv import ArmControl

# M State,Angle,Deg/Step, 
class MotorController(Node):
  def __init__(self):
    # Sub to get M state
    self.create_service(DynamixelStateList, 
                        '/dynamixel_workbench/dynamixel_state',
                        self.getMotorStateCB)
    # Pub to ctrl M
    self.m_pub = self.create_publisher(Float64MultiArray,
                                           'dynamixel_workbench/joint_trajectory',
                                           queue_size = 10)
    # Pub to send M angle
    self.m_angle_pub = self.create_publisher(Float64MultiArray,
                                                 '/servo/angle_list',
                                                 queue_size = 10)
    timer_period=0.5; self.create_timer(timer_period, self.motorAnglePub)
    # ServiceClient to ctrl M
    self.m_client = self.create_client(DynamixelCommand,
                                           '/dynamixel_workbench/dynamixel_command')
    while not self.m_client.wait_for_service(1.0):
      self.get_logger().warn('waiting for service... /dynamixel_workbench/dynamixel_command')
    dxl_cmd_future = self.m_client.call_async(request)
    rclpy.spin_until_future_complete(self, dxl_cmd_future)
    # M parameters
    self.origin_angle = self.declare_parameter('/mimi_specification/Origin_Angle')
    self.gear_ratio = self.declare_parameter('/mimi_specification/Gear_Ratio')
    self.latest_pose = [0]*6 # M step values
    self.torque_error = [0]*6 # M current values
    self.rotation_velocity = [0]*6 # M angle values
    
  # save each latest M state(step posi, velo, curr)
  def getMotorStateCB(self, state): # type: DynamixelStateList
    for i in range(len(state.dynamixel_state)): # (6 elements)
      self.latest_pose[i] = state.dynamixel_state[i].present_position
      self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity) #abs velo
      self.torque_error[i] = state.dynamixel_state[i].present_current # curr
     
  # pub M Angle at rate of 2Herz
  def motorAnglePub(self, event): #!!! event?
    # steps to degs
    origin_angle_deg = list(map(self.stepToDeg, self.origin_angle))
    latest_pose_deg = list(map(self.stepToDeg, self.latest_pose))
    # !!!    [process-append-num for i,j in list(),list() process]
    latest_deg_list = [x-y for (x,y) in zip(latest_pose_deg, origin_angle_deg)] #!!!
    latest_deg_list = [round(x, 1) for x in latest_deg_list]
    latest_deg_list[2] *= -1
    latest_deg_list[5] *= -1
    pub_deg_list = Float64MultiArray(data=latest_deg_list)    
    self.motor_angle_pub.publish(pub_deg_list)
  
# ---------------------
  def degToStep(self, deg):
    return int( (deg + 180) / 360.0  * 4095 ) # add 180 deg for dxl origin posi
  
  def stepToDeg(self, step):
    return round( step / 4095.0 * 360.0 - 180, 1) # minus 180
  
  # move M
  def motorPub(self, joint_name, joint_angle, execute_time = 0.8):
    msg = JointTrajectory()
    # JointTrajectory.msg Header header; string[] joint_names; JointTrajectoryPoint[] points
    # JointTrajectoryPoint.msg
    # float64[] positions, velocities, accelerations, effort; duration time_from_start
    t = self.set_clock().now
    msg.header.stamp = t.to_msg() #!!!
    msg.Joint_names = joint_name # <- string[]
    #msg.points = [JointTrajectoryPoint() for i in range(1)]
    msg.points = [JointTrajectoryPoint()]
    msg.points[0].positions = joint_angle  # target angle = float64[posi[ƒÆ]] <---
    msg.points[0].time_from_start = time.time(execute_time)  # !!!
    self.motor_pub.publish(msg) 
  
  # set M posi(ether steps or deg value) and send dxl service client with them.
  def setPosition(self, m_id, position_value):
    if type(position_value) == type(float()):
      rotate_value = self.degToStep(position_value) # !!!
    res = self.m_client('', m_id, 'Goal_Position', position_value)
      
  # set a allowable current value to protect againist too big load
  def setCurrent(self, m_id, current_value):
    res = self.motor_client('', m_id, 'Goal_Current', current_value)
    
# ctrl eath M 
class JointController(MotorController):
  def __init__(self):
    super(JointController,self).__init__()
    self.create_subscription('/servo/shoulder',Float64,self.controlShoulder)
    self.create_subscription('/servo/elbow',Float64,self.controlElbow)
    self.create_subscription('/servo/wrist',Float64,self.controlWrist)
    self.create_subscription('/servo/endeffector',Bool,self.controlEndeffector)
    self.create_subscription('/servo/head',Float64,self.controlHead) # Range: -30~40[deg]
    
  # convert ratate value of shoulder M. type of "deg" is able to be float or int or other
  def shoulderConversionProcess(self, deg):
    deg *= self.gear_ratio[0]
    rad = math.radians(deg) # deg to radian
    print('rad:', rad)
    m0_rad = -1 * rad + self.stepToRad(self.origin_angle[0]) # "-1"
    m1_rad = rad + self.stepToRad(self.origin_angle[1])
    print('m0_origin', self.stepToRad(self.origin_angle[0]))
    print('m1_origin', self.stepToRad(self.origin_angle[1]))
    return m0_rad, m1_rad
    
  def elbowConversionProcess(self, deg):
    rad = math.radians(deg)                                   
    m2_rad = rad + self.stepToRad(self.origin_angle[2])       
    print('m2_origin', self.stepToRad(self.origin_angle[2]))  
    return m2_rad
  
  def wristConversionProcess(self, deg):
    rad = math.radians(deg)
    m3_rad = rad + self.stepToRad(self.origin_angle[3])
    print('m3_origin', self.stepToRad(self.origin_angle[3]))
    return m3_rad
  
  def controlShoulder(self,deg): # '/servo/shoulder', Float64
    try:
      deg = deg.data
    except AttributeError:
      pass
    m0, m1 = self.shoulderConversionProcess(deg)  # 
    self.motorPub(['m0_shoulder_left_joint', 'm1_shoulder_right_joint'], [m0, m1])
  
  def controlElbow(self, deg):
    try:
      deg = deg.data
    except AttributeError:
      pass
    m2 = self.elbowConversionProcess(deg)
    self.motorPub(['m2_elbow_joint'], [m2])
    
  def controlWrist(self,deg):
    try:
      deg = deg.data
    except AttributeError:
      pass
    m3 = self.wristConversionProcess(deg)
    self.motorPub(['m3_wrist_joint'], [m3])
    
  # '/servo/endeffector',Bool
  def controlEndeffector(self, req):
    try:
      req = req.data
    except AttributeError: # !!!
      pass
    
    # OPEN
    rate = self.create_rate[5]
    if not req: # if eef is close
      self.setCurrent(4, 200) # set allowable curr
      self.setPosition(4, self.origin_angle[4])
      rate.sleep()
      return True

    # CLOSE
    c_rate = self.create_rate[2]
    goal_position = self.origin_angle[4] + 480
    self.setCurrent(4. 200)
    self.setPosition(4, goal_position)
    c_rate.sleep()
    # in case of eef anglar velo ia more than 0 and 
    while self.rotation_velocity[4] > 0 and not rclpy.shutdown():
      pass # notiong to do
    else:
      c_rate.sleep()
      #self.setPosition(4, self.latest_pose[4])
      # !!!
    grasp_flg = self.torque_error[4] > 30 # whetherif eef M curr is more than 30
    # !!! why 30?
    print(grasp_flg)
    return grasp_flg
  
  def controlHead(self, deg):
    try:
      deg = deg.data
    except AttributeError:
      pass
    deg *= self.gear_ratio[5]
    step = self.degToStep(deg)  +  (self.origin_angle[5] - 2048) # deg + ori-180
    # !!! why is stepValue used in only head ratating
    self.setPosition(5, step)
  
class ManipulateArm(JointController):
  def __init__(self):
    


def main(args=None):
  rclpy.init(args=args)  # initialize node
  node = Node("motor_controller2")

  mami = ManipulateArm()
  rclpy.spin(node)
  
if __name__ == '__main__':
  main()