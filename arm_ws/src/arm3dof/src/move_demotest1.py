#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class ArmController(Node):
  def __init__(self):

    self.current_target_index = 1
    self.wait_start_time =None

    super().__init__('arm_controller')

    self.pub_j1 = self.create_publisher(Float64, 'cmd_pos_j1', 10)
    self.pub_j2 = self.create_publisher(Float64, 'cmd_pos_j2', 10)
    self.pub_j3 = self.create_publisher(Float64, 'cmd_pos_j3', 10)

    self.target_poses = [
      [0.0, 0.0, 0.0],
      [1.57, 0.5, 0.5],
      [-1.57, 0.5, 0.5],
      [0.0, 0.8, -0.8]
    ]
    

    
    self.current_pos_j1 = self.target_poses[self.current_target_index - 1][0]
    self.current_pos_j2 = self.target_poses[self.current_target_index - 1][1]
    self.current_pos_j3 = self.target_poses[self.current_target_index - 1][2]

    self.timer = self.create_timer(0.01, self.timer_call_back)
    self.get_logger().info("Robot contoller has been started...")


  def timer_call_back(self):

    self.step_size = 0.005

    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()


    

    if (self.target_poses[self.current_target_index][0] - self.current_pos_j1 < 0):
      if (self.current_pos_j1 - self.step_size > self.target_poses[self.current_target_index][0]):
        self.current_pos_j1 -= self.step_size
        msg1.data = self.current_pos_j1
        self.pub_j1.publish(msg1)
      else:
        self.current_pos_j1 = self.target_poses[self.current_target_index][0]
        msg1.data = self.current_pos_j1
        self.pub_j1.publish(msg1)

    else:
      if (self.current_pos_j1 + self.step_size < self.target_poses[self.current_target_index][0]):
        self.current_pos_j1 += self.step_size
        msg1.data = self.current_pos_j1
        self.pub_j1.publish(msg1)
      else:
        self.current_pos_j1 = self.target_poses[self.current_target_index][0]
        msg1.data = self.current_pos_j1
        self.pub_j1.publish(msg1)



    if (self.target_poses[self.current_target_index][1] - self.current_pos_j2 < 0):
      if (self.current_pos_j2 - self.step_size > self.target_poses[self.current_target_index][1]):
        self.current_pos_j2 -= self.step_size
        msg2.data = self.current_pos_j2
        self.pub_j2.publish(msg2)
      else:
        self.current_pos_j2 = self.target_poses[self.current_target_index][1]
        msg2.data = self.current_pos_j2
        self.pub_j2.publish(msg2)
    else:
      if (self.current_pos_j2 + self.step_size < self.target_poses[self.current_target_index][1]):
        self.current_pos_j2 += self.step_size
        msg2.data = self.current_pos_j2
        self.pub_j2.publish(msg2)
      else:
        self.current_pos_j2 = self.target_poses[self.current_target_index][1]
        msg2.data = self.current_pos_j2
        self.pub_j2.publish(msg2)
        




    if (self.target_poses[self.current_target_index][2] - self.current_pos_j3 < 0):
      if (self.current_pos_j3 - self.step_size > self.target_poses[self.current_target_index][2]):
        self.current_pos_j3 -= self.step_size
        msg3.data = self.current_pos_j3
        self.pub_j3.publish(msg3)
      else:
        self.current_pos_j3 = self.target_poses[self.current_target_index][2]
        msg3.data = self.current_pos_j3
        self.pub_j3.publish(msg3)

    else:
      if (self.current_pos_j3 + self.step_size < self.target_poses[self.current_target_index][2]):
        self.current_pos_j3 += self.step_size
        msg3.data = self.current_pos_j3
        self.pub_j3.publish(msg3)
      else:
        self.current_pos_j3 = self.target_poses[self.current_target_index][2]
        msg3.data = self.current_pos_j3
        self.pub_j3.publish(msg3)


    if msg1.data == self.target_poses[self.current_target_index][0] and msg2.data == self.target_poses[self.current_target_index][1] and msg3.data == self.target_poses[self.current_target_index][2]:
      
      if self.wait_start_time is None:
        self.wait_start_time = time.time()
        self.get_logger().info("Hedefe varildi siradaki hedef icin bekleniyor")
      
      passed_time = time.time() - self.wait_start_time

      if passed_time >= 3.0:

        self.get_logger().info("Siradaki hedefe geciliyor...")

        if self.current_target_index == 3:
          self.current_target_index = 0
        else:
          self.current_target_index +=1
        self.wait_start_time = None
      
    else:
        self.wait_start_time = None


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