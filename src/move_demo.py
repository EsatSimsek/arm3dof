#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class ArmController(Node):
  def __init__(self):

    super().__init__('arm_controller')

    self.step_size = 0.01
    self.wait_time = 3.0


    self.pub_j1 = self.create_publisher(Float64, 'cmd_pos_j1', 10)
    self.pub_j2 = self.create_publisher(Float64, 'cmd_pos_j2', 10)
    self.pub_j3 = self.create_publisher(Float64, 'cmd_pos_j3', 10)

    self.target_poses = [
      [0.0, 0.0, 0.0],
      [1.57, 0.5, 0.5],
      [-1.57, 0.5, 0.5],
      [0.0, 0.8, -0.8]
    ]

    self.current_target_index = 0
    self.current_joints = [0.0, 0.0, 0.0]

    self.wait_counter = 0

    

    self.timer = self.create_timer(0.02, self.timer_call_back)
    self.get_logger().info("Robot baslatildi.")


  def timer_call_back(self):

    target = self.target_poses[self.current_target_index]

    arrived = True

    for i in range(3):
      if abs(self.current_joints[i] - target[i]) > 0.01:
        arrived = False
        break

    if arrived == True:
      self.get_logger().info("Hedefe varildi. Siradaki hedef icin bekleniyor")
      self.wait_counter += 0.02
      if self.wait_counter >= self.wait_time:
        self.wait_counter = 0
        self.current_target_index = (self.current_target_index + 1) % 4
        

    else:
      self.get_logger().info("Siradaki hedefe gidiliyor")
      for i in range(3):
        diff = target[i] - self.current_joints[i]
        
        if abs(diff) > self.step_size:
          if diff > 0:
            self.current_joints[i] += self.step_size
          else:
            self.current_joints[i] -= self.step_size
        else:
          self.current_joints[i] = target[i]

    self.publish_all()


  def publish_all(self):

    msg1= Float64()
    msg2 = Float64()
    msg3 = Float64()

    msg1.data = self.current_joints[0]
    msg2.data = self.current_joints[1]
    msg3.data = self.current_joints[2]


    self.pub_j1.publish(msg1)
    self.pub_j2.publish(msg2)
    self.pub_j3.publish(msg3)



def main():
  rclpy.init()
  arm_controller_node = ArmController()
  rclpy.spin(arm_controller_node)
  arm_controller_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
else:
  pass