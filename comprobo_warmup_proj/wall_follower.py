import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from visualization_msgs.msg import Marker

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.create_timer(0.1, self.act)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.store_scan, 10)
        self.marker_pub = self.create_publisher(Marker, 'wall_marker', 10)
        
        self.scan = {}

    def act(self):
        error = self.get_error()
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = error * -0.5

        self.vel_pub.publish(cmd_vel)
        self.pub_marker()


    def get_error(self):
        back_scan = [self.scan[x] if x in self.scan else np.inf for x in range(220, 230)]
        front_scan = [self.scan[x] if x in self.scan else np.inf for x in range(310, 320)]
        differences = [front_scan[x] - back_scan[x] for x in range(len(back_scan))]
        
        output = sum(differences)/len(differences)

        if np.isnan(output): output = 0.0

        return output

    def pub_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.g = 1.0

        right_avg = [self.scan[x] for x in range(267, 273) if x in self.scan]
        if not right_avg: right_avg = [0]
        right_avg = sum(right_avg) / len(right_avg)

        marker.pose.position.y = -right_avg

        self.marker_pub.publish(marker)

    def store_scan(self, msg):
        self.scan = {i:j for i,j in enumerate(msg.ranges) if j not in [0.0, np.inf]}

    def get_now(self):
        return self.get_clock().now().to_msg().sec


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()