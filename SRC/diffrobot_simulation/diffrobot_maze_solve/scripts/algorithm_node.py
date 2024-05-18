import time 
import rclpy
from diffrobot_maze_solve.moving_node import RobotMoving

robotcontrol = RobotMoving()

class Robot:

  def __init__(self):
    self.laser1 = robotcontrol.get_laser(360)
    self.robotmove_speed = 5
    self.robotmove_time = 1
    #self.robotturn_clockwise = "clockwise"
    self.robotturn_speed = 0.8
    self.robotturn_time = 1
  
    while True:
        
        self.laser1 = robotcontrol.get_laser(360)
        distance_right = robotcontrol.get_laser(90)  # Right distance
        distance_front = self.laser1
        
        if distance_right > 1:
            # Turn right if there's space
            while distance_front <= 1:
                robotcontrol.turn("clockwise", self.robotturn_speed, self.robotturn_time)
                distance_front = robotcontrol.get_laser(360)
                print("Turning right, front distance:", distance_front)
        elif distance_front > 1:
            # Move straight if there's no obstacle in front
            robotcontrol.move_straight()
            print("Moving straight, front distance:", distance_front)
        else:
            # Turn left if there's no space to the right and there's an obstacle in front
            while distance_front <= 1:
                robotcontrol.turn("anticlockwise", self.robotturn_speed, self.robotturn_time)
                distance_front = robotcontrol.get_laser(360)
                print("Turning left, front distance:", distance_front)
                
        robotcontrol.stop_robot()
        print("stop")

def main(args=None):
  rclpy.init(args=args)
  robot = Robot()
  
  try:
    while rclpy.ok():
      robot.robotmove()
      robot.robotturn()
  except KeyboardInterrupt:
    pass

  robot.destroy_node()
  rclpy.shutdown()


if __name__ =='__main__':
  main()