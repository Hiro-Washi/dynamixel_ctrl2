#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Bool, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand
# -- Custom Message --
from happymimi_msgs2.srv import StrTrg
from happymimi_recognition_msgs2.srv import PositionEstimator
from mimi_mani_msgs2.srv import ArmControl

class MotorController(Node):
  def __init__(self):
    super().__init__('motor_controller2')
    # ServiceServer
    self.create_service(DynamixelStateList, 
                        '/dynamixel_workbench/dynamixel_state',
                        self.getMotorStateCB)
    # ServiceClient
    self.m_client = self.create_client(DynamixelCommand,
                                           '/dynamixel_workbench/dynamixel_command')
    while not self.m_client.wait_for_service(1.0):
      self.get_logger().info('waiting for the service... /dynamixel_workbench/dynamixel_command')
    self.dxl_req = DynamixelCommand.Request()
    # Pub
    self.m_pub = self.create_publisher(Float64MultiArray,
                                           'dynamixel_workbench/joint_trajectory',
                                           queue_size = 10)
    # Pub
    self.m_angle_pub = self.create_publisher(Float64MultiArray,
                                                 '/servo/angle_list',
                                                 queue_size = 10)
    # PeriodicPub
    timer_period=0.5; self.create_timer(timer_period, self.motorAnglePub)
    # M parameters
    self.origin_angle = self.declare_parameter('/mimi_specification/Origin_Angle')
    self.gear_ratio = self.declare_parameter('/mimi_specification/Gear_Ratio')
    self.latest_pose = [0]*6
    self.torque_error = [0]*6
    self.rotation_velocity = [0]*6
    
  def getMotorStateCB(self, state):
    for i in range(len(state.dynamixel_state)):
      self.latest_pose[i] = state.dynamixel_state[i].present_position
      self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity)
      self.torque_error[i] = state.dynamixel_state[i].present_current
     
  def motorAnglePub(self, event):
    origin_angle_deg = list(map(self.stepToDeg, self.origin_angle))
    latest_pose_deg = list(map(self.stepToDeg, self.latest_pose))
    latest_deg_list = [x-y for (x,y) in zip(latest_pose_deg, origin_angle_deg)]
    latest_deg_list = [round(x, 1) for x in latest_deg_list]
    latest_deg_list[2] *= -1
    latest_deg_list[5] *= -1
    pub_deg_list = Float64MultiArray(data=latest_deg_list)    
    self.m_pub.publish(pub_deg_list)

  def degToStep(self, deg):
    return int( (deg + 180) / 360.0  * 4095 )
  
  def stepToDeg(self, step):
    return round( step / 4095.0 * 360.0 - 180, 1)
  
  def motorPub(self, joint_name, joint_angle, execute_time = 0.8):
    msg = JointTrajectory()
    #t = self.set_clock().now
    msg.header.stamp = Clock.now #t.to_msg()
    msg.Joint_names = joint_name
    #msg.points = [JointTrajectoryPoint() for i in range(1)]
    msg.points = [JointTrajectoryPoint()]
    msg.points[0].positions = joint_angle
    msg.points[0].time_from_start = time.time(execute_time)
    self.m_pub.publish(msg) 
  
  def dxlCallAsync(self, cmd, id, name, value):
    self.dxl_req.command = cmd
    self.dxl_req.id = id
    self.dxl_req.addr_name = name
    self.dxl_req.value = value
    return self.m_client.call_async(self.dxl_req)
  
  def setPosition(self, m_id, position_value):
    if type(position_value) == type(float()):
      rotate_value = self.degToStep(position_value)
    dxl_future = self.dxlCallAsync('', m_id, 'Goal_Position', position_value)
    rclpy.spin_until_future_complete(self, dxl_future)
    result = dxl_future.result()
    #return result # type: Bool

  def setCurrent(self, m_id, current_value):
    dxl_future = self.dxlCallAsync('', m_id, 'Goal_Current', current_value)
    rclpy.spin_until_future_complete(self, dxl_future)
    result = dxl_future.result()
    #return result # type: Bool

class JointController(MotorController):
  def __init__(self):
    super(JointController,self).__init__()
    self.create_subscription(Float64, '/servo/shoulder', self.controlShoulder)
    self.create_subscription(Float64, '/servo/elbow', self.controlElbow)
    self.create_subscription(Float64, '/servo/wrist', self.controlWrist)
    self.create_subscription(Bool,    '/servo/endeffector', self.controlEndeffector)
    self.create_subscription(Float64, '/servo/head', self.controlHead) # Range:-30~40[deg]

  def shoulderConversionProcess(self, deg):
    deg *= self.gear_ratio[0]
    rad = math.radians(deg)
    print('rad:', rad)
    m0_rad = -1 * rad + self.stepToRad(self.origin_angle[0])
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
  
  def controlShoulder(self,deg):
    try:
      deg = deg.data
    except AttributeError:
      pass
    m0, m1 = self.shoulderConversionProcess(deg)
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
    
  def controlEndeffector(self, req):
    try:
      req = req.data
    except AttributeError:
      pass
    # OPEN
    rate = self.create_rate(5)
    if not req:
      self.setCurrent(4, 200)
      self.setPosition(4, self.origin_angle[4])
      rate.sleep()
      return True
    # CLOSE 
    c_rate = self.create_rate(2)
    goal_position = self.origin_angle[4] + 480
    self.setCurrent(4, 200)
    self.setPosition(4, goal_position)
    c_rate.sleep()
    while self.rotation_velocity[4] > 0 and not rclpy.shutdown():
      pass
    else:
      c_rate.sleep()
      #self.setPosition(4, self.latest_pose[4])
    grasp_flg = self.torque_error[4] > 30
    print(grasp_flg)
    return grasp_flg
  
  def controlHead(self, deg):
    try:
      deg = deg.data
    except AttributeError:
      pass
    deg *= self.gear_ratio[5]
    step = self.degToStep(deg) + (self.origin_angle[5] - 2048)
    self.setPosition(5, step)
  
class ManipulateArm(JointController):
  def __init__(self):
    super(ManipulateArm, self).__init__()
    self.create_service(StrTrg, '/servo/arm', self.changeArmPose)
    # ServiceServer
    self.create_service( ArmControl, '/servo/debug_arm', self.armControlService) #InverseKinetics
    # ServiceClient
    self.depth_client = self.create_client(PositionEstimator, '/detect/depth')
    # Param
    self.arm_specification = self.declare_parameter('/mimi_specification')

  def inverseKinematics(self, coordinate):
    x  = coordinate[0]
    y  = coordinate[1]
    l0 = self.arm_specification['Ground_Arm_Height']
    l1 = self.arm_specification['Shoulder_Elbow_Length']
    l2 = self.arm_specification['Elbow_Wrist_Length']
    l3 = self.arm_specification['Wrist_Endeffector_Length']
    x -= l3
    y -= l0
    data1 = x*x + y*y + l1*l1 - l2*l2
    data2 = 2 * l1 * math.sqrt(x*x + y*y)
    try:
      shoulder_angle = math.atan(y/x) - math.acos(data1 / data2 + math.atan(y/x))
      elbow_angle = math.atan( (y- l1*math.sin(shoulder_angle)) / (x- l1*math.cos(shoulder_angle)))-shoulder_angle
      wrist_angle = -1 * (shoulder_angle + elbow_angle)
      angle_list = map(math.degrees, angle_list)
      return angle_list
    except ValueError:
      self.get_logger().info("can't move arm.")
      return [numpy.nan]*3

  # we don't use this.
  # def armController(self, joint_angle):
  #   try:
  #     joint_angle = joint_angle.data
  #   except AttributeError:
  #     pass
  #   thread_shoulder = threading.Thread(target=self.controlShoulder, args=(joint_angle[0],))
  #   thread_elbow = threading.Thread(target=self.controlElbow, args=(joint_angle[1],))
  #   thread_wrist = threading.Thread(target=self.controlWrist, args=(joint_angle[2],))
  #   rate = self.create_rate(2)
  #   thread_wrist.start()
  #   rate.sleep()
  #   thread_elbow.start()
  #   rate.sleep()
  #   thread_shoulder.start()
  
  def armControllerByTopic(self, joint_angle):
    m0, m1 = self.shoulderConversionProcess(joint_angle[0])
    m2 = self.elbowConversionProcess(joint_angle[1])
    m3 = self.wristConversionProcess(joint_angle[2])
    print('m0, m1, m2, m3')
    print(m0, m1, m2, m3)
    print(map(math.degrees, [m0, m1, m2, m3]))
    # mani mani
    self.motorPub(['m0_shoulder_left_joint', 'm1_shoulder_right_joint',\
                   'm2_elbow_joint', 'm3_wrist_joint'], [m0, m1, m2, m3])

  def armControlService(self, coordinate):
    try:
      coordinate = coordinate.data
    except AttributeError:
      pass
    joint_angle = self.inverseKinematics(coordinate)
    print('')
    print('joint_angle')
    print(joint_angle)
    if numpy.nan in joint_angle:
      return False
    #self.armController(joint_angle)
    self.armControllerByTopic(joint_angle)
    return True

  def changeArmPose(self, cmd):
    if type(cmd) != str:
      cmd = cmd.data
    self.get_logger().info("Change arm command : %s"%cmd)
    if cmd == 'origin':
      self.originMode()
      return True
    elif cmd == 'carry':
      self.carryMode()
      return True
    elif cmd == 'receive':
      res = self.receiveMode()
      return res
    elif cmd == 'give':
      self.giveMode()
      return True
    elif cmd == 'place':
      res = self.placeMode()
      return res
    else :
      self.get_logger_info('No such change arm command.')
      return False
  
  def originMode(self):
    shoulder_param = 0
    elbow_param = 0
    wrist_param = 0
    #self.armController([shoulder_param, elbow_param, wrist_param])
    self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])
    
  def carryMode(self):
    shoulder_param = -85
    elbow_param = 90
    wrist_param = 90
    #self.armController([shoulder_param, elbow_param, wrist_param])
    self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])

  def receiveMode(self):
    self.controlHead(25)
    rate_2 = self.create_rate(2); rate_2.sleep()
    rate_05 = self.create_rate(0.5)
    shoulder_param = -40
    elbow_param = 70
    wrist_param = -30
    #self.armController([shoulder_param, elbow_param, wrist_param])
    self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])
    rate_2.sleep()
    self.controlEndeffector(False) # OPEN
    while not self.detect_depth.wait_for_service(timeout_sec= 1.0):
      self.get_logger().info('wait for the service... /detect/depth')
    self.depth_req = PositionEstimator.Request()
    endeffector_result = False
    count = 0
    '''
    while not endeffector_res and count<2 and not rclpy.shutdown():
      self.controlEndeffector(False) # OPEN eef
      rate_05.sleep()
      count += 1
      start_time = time.time()
      straight_line_distance = 9.99
      while time.time()-start_time<3.0 and straight_line_distance>0.42 and not rclpy.shutdown():
        #depth_res = self.detect_depth(280, 360) # !!!
            #srv/PositionEstimator: int64 center_x; int64 center_y
            #                       geometry_msgs/Point point
        self.depth_req.center_x = 280
        self.depth_req.center_y = 360
        depth_future = self.depth_client.call_async(self.depth_req)
        rclpy.spin_until_future_complete(self, depth_future)
        depth_result = depth_future.result()
        straight_line_distance = depth_res.point.x #!!!
      rate_2.sleep()
      endeffector_res = self.controlEndeffector(True) # CLOSE eef
      rate_05.sleep()
    '''
    self.controlEndeffector(False) # OPEN
    rate_05.sleep()
    start_time = time.time()
    straight_line_distance = 9.99
    while time.time()-start_time<3.0 and straight_line_distance>0.42 and not rclpy.shutdown():
      self.depth_req.center_x = 280; self.depth_req.center_y = 365
      depth_future = self.depth_client.call_async(self.depth_req)
      rclpy.spin_until_future_complete(self, depth_future)
      depth_result = depth_future.result()
      straight_line_distance = depth_result.point.x
    rate_2.sleep()
    endeffector_result = self.controlEndeffector(True) # GRASP result
    rate_05.sleep()
      
    self.carryMode()
    self.controlHead(0)
    return endeffector_result

  def giveMode(self):
    shoulder_param = -35
    elbow_param = 75
    wrist_param = -35
    #self.armController([shoulder_param, elbow_param, wrist_param])
    self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])
    rate = self.create_rate(0.25); rate.speep()
    self.get_logger().info("give!!")
    '''
      while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
        pass
      rate_1 = self.create_rate(1.0); rate_1.sleep()
    '''
    wrist_error = abs(self.torque_error[3])
    give_time = time.time()
    while abs(wrist_error - abs(self.torque_error[3])) < 10 and time.time()-give_time<5.0 and not rclpy.shutdown():
      pass
    self.setPosition(4, self.origin_angle[4]) # OPEN
    rate = self.create_rate(2); rate.sleep()
    self.carryMode()
  
  def placeMode(self):
    pass
    
def main(args=None):
  rclpy.init(args=args)
  mani_arm = ManipulateArm()
  rclpy.spin(mani_arm)
  
if __name__ == '__main__':
  main()