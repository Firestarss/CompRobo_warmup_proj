import tty
import select
import sys
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.setVel)
        self.settings = termios.tcgetattr(sys.stdin)

        self.msg = Twist()

    def setVel(self):
        key = self.getKey()

        # End program if Ctrl+c or ESC are pressed
        if key == '\x03' or ord(key) == 27: 
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        elif key == "w":
            self.msg.linear.x += 0.5
        elif key == "a":
            self.msg.angular.z += 0.5
        elif key == "s":
            self.msg.linear.x -= 0.5
        elif key == "d":
            self.msg.angular.z -= 0.5
        elif key == " ":
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0

        self.publisher_.publish(self.msg)

        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    print("w/s: increment/decrement linear velocity")
    print("a/d: increment/decrement angular velocity")
    print("[Space] to reset linear/angular velocities to 0")
    print("[ESC] or Ctrl-c to exit\n")
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)

if __name__ == '__main__':
    main()