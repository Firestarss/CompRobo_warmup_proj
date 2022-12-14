import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DriveSquareNode(Node):
    """
    ROS2 Node to make a Neato drive in a roughly 1m x 1m square
    """
    def __init__(self):
        """
        Initialize Node and set up publishers and instance variables
        """
        # initialize ROS2 node and create a publisher to control the Neato
        super().__init__('drive_square_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # create default Twist messges to move Neato forward/turn
        self.forward_twist = Twist()
        self.forward_twist.linear.x = 0.3

        self.turn_twist = Twist()
        self.turn_twist.angular.z = 0.3

        # set up state tracker
        self.state = "turn"

        # initialize variable to track when a leg ends
        self.end_time = self.get_numerical_time() - 1

        # initialize variables for how long each let should be
        self.forward_time = 3.84
        self.turn_time = 5.05

        # initialize variable to track which leg the Neato is on
        self.leg = 0

        # initialize timer to run move function at regular intervals
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.move)

    def move(self):
        """
        Function that takes the current state of the Node and publises the corresponding
        pre-defined Twist message
        """
        # check of the current leg has finished
        if self.get_numerical_time() >= self.end_time:
            # incriment the leg number by one (caps at 9)
            self.leg = min(self.leg + 1, 9)
            # if the Neato has finished the last leg, stop it and set state to done
            if self.state != "done" and self.leg > 8:
                self.publisher.publish(Twist())
                self.state = "done"
            # if the Neato is done, do noting
            elif self.state == "done":
                pass
            # if the Neato finished turning, increment the leg number and make it drive forward
            elif self.state == "turn":
                self.state = "forward"
                self.end_time = self.get_numerical_time() + self.forward_time
                self.publisher.publish(self.forward_twist)  
            # if the Neato finished driving forward, increment the leg number and make it turn
            else:
                self.state = "turn"
                self.end_time = self.get_numerical_time() + self.turn_time
                self.publisher.publish(self.turn_twist)   

    def get_numerical_time(self):
        """
        returns the time in seconds. Includes nano seconds as a decimal
        example return: 1671004232.35
        """
        # gets the current time in the form of a ROS message
        msg = self.get_clock().now().to_msg()
        # converts nanoseconds to seconds
        nano_dec = msg.nanosec / 10e9
        # retturns the time in seconds
        return msg.sec + nano_dec

def main(args=None):
    """
    Initializes ROS2 and control Node
    """
    rclpy.init(args=args)
    drive_square_node = DriveSquareNode()
    rclpy.spin(drive_square_node)

if __name__ == "__main__":
    main()