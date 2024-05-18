import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotMoving():

  def __init__(self):

    super().__init__('robot_control_node')
    self.vel_publisher = self.create_publisher(Twist,'/cmd_vel', 1)
    self.laser_subscriber = self.create_subscription(LaserScan,'/laser_scan', self.laser_callback,10)
    self.cmd = Twist()
    self.laser_msg = LaserScan()
    self.ctrl_c = False
    self.rate = self.create_rate(1)

  def publish_once_in_cmd_vel(self):
    while rclpy.ok () and not self.ctrl_c:
      connections = self.vel_publisher.get_subscription_count()
      if connections > 0:
        self.vel_publisher.publish(self.cmd)
        break
      else:
        self.rate.sleep()

  def shutdown(self):
    self.ctrl_c = True

  def laser_callback(self, msg):
    self.laser_msg = msg

  def get_laser(self,pos):
    time.sleep(1)
    return self.laser_msg_ranges[pos]

  def get_front_laser(self):
    time.sleep(1)
    return self.laser_msg.ranges[360]
  
  def get_laser_full(self):
    time.sleep(1)
    return self.laser_msg.ranges
  
  def stop_robot(self):
    self.cmd.linear.x = 0.0
    self.cmd.angular.z = 0.0
    self.publish_once_in_cmd_vel()

  def move_straight(self):

    self.cmd.linear.x = 0.5
    self.cmd.linear.y = 0
    self.cmd.linear.z = 0
    self.cmd.angular.x = 0
    self.cmd.angular.y = 0
    self.cmd.angular.z = 0

    self.publish_once_in_cmd_vel()

  def move_straight_time(self, motion, speed, time):

    self.cmd.linear.y = 0
    self.cmd.linear.z = 0
    self.cmd.angular.x = 0
    self.cmd.angular.y = 0
    self.cmd.angular.z = 0

    if motion == "forward":
      self.cmd.linear.x = speed
    elif motion == "backward":
      self.cmd.linear.x = -speed

    start_time = time.time()

    elapsed_time = 0
    while elapsed_time <= time:
      # Publish the velocity
      self.vel_publisher.publish(self.cmd)

      # Calculate elapsed time
      elapsed_time = time.time() - start_time

      # Sleep to maintain loop frequency
      self.rate.sleep()

    # Stop the robot
    self.stop_robot()

    s = "Moved robot " + motion + " for " + str(time) + " seconds"
    return s
  
def turn(self, clockwise, speed, time):
  self.cmd.linear.x = 0
  self.cmd.linear.y = 0
  self.cmd.linear.z = 0
  self.cmd.angular.x = 0
  self.cmd.angular.y = 0

  if clockwise == "clockwise":
    self.cmd.linear.z = speed
  else:
    self.cmd.linear.z = -speed

  start_time = time.time()

  elapsed_time = 0
  while elapsed_time <= time:
    # Publish the velocity
    self.vel_publisher.publish(self.cmd)

    # Calculate elapsed time
    elapsed_time = time.time() - start_time

    # Sleep to maintain loop frequency
    self.rate.sleep()

  # Stop the robot
  self.stop_robot()

  s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
  return s

if __name__=='__main__':
  robotcontrol_object = RobotMoving()
  