import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.create_timer(0.1, self.act)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bumper_active = False
        self.state = "forwards"
        self.end_time = self.get_now()

        msg = Twist()
        msg.linear.x = 0.8
        self.vel_pub.publish(msg)
    
    def act(self):
        if self.state == "forwards" and self.bumper_active:
            self.state = "backwards"
            self.end_time = self.get_now() + 2

            msg = Twist()
            msg.linear.x = -0.8
            self.vel_pub.publish(msg)
        
        elif self.state == "backwards" and self.get_now() > self.end_time:
            self.state = "forwards"
            msg = Twist()
            msg.linear.x = 0.8
            self.vel_pub.publish(msg)
    
    def process_bump(self, msg):
        self.bumper_active = 1 in [msg.left_front, msg.left_side,
                                   msg.right_front, msg.right_side]

    def get_now(self):
        return self.get_clock().now().to_msg().sec


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()