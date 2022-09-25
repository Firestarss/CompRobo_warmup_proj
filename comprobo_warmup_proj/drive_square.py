import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.forward_twist = Twist()
        self.forward_twist.linear.x = 0.3

        self.turn_twist = Twist()
        self.turn_twist.angular.z = 0.3

        self.state = "turn"
        self.end_time = self.get_numerical_time() - 1

        self.forward_time = 3.84
        self.turn_time = 5.05

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.move)

        self.leg = 0

    def move(self):
        if self.get_numerical_time() >= self.end_time:
            self.leg = min(self.leg + 1, 9)
            if self.state != "done" and self.leg > 8:
                self.publisher.publish(Twist())
                self.state = "done"
            elif self.state == "done":
                pass
            elif self.state == "turn":
                self.state = "forward"
                self.end_time = self.get_numerical_time() + self.forward_time
                self.publisher.publish(self.forward_twist)                
            else:
                self.state = "turn"
                self.end_time = self.get_numerical_time() + self.turn_time
                self.publisher.publish(self.turn_twist)   

    def get_numerical_time(self):
        msg = self.get_clock().now().to_msg()
        nano_dec = msg.nanosec / 10e9
        return msg.sec + nano_dec

def main(args=None):
    rclpy.init(args=args)
    drive_square_node = DriveSquareNode()
    rclpy.spin(drive_square_node)

if __name__ == "__main__":
    main()