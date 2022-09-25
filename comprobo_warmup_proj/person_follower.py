import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from visualization_msgs.msg import Marker


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.create_timer(0.1, self.act)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.store_scan, 10)
        self.marker_pub = self.create_publisher(Marker, 'person_marker', 10)
        self.width = 10
        self.depth = 10
        
        self.scan = {}

    def act(self):
        person = self.get_person()
        cmd_vel = Twist()
        cmd_vel.linear.x = person[0] * 0.2
        cmd_vel.angular.z = person[1] * 0.5

        self.vel_pub.publish(cmd_vel)
        self.pub_marker(person)


    def get_person(self):
        left_scan = {x:self.scan[x] if x in self.scan else np.inf for x in range(90)}
        right_scan = {x:self.scan[x] if x in self.scan else np.inf for x in range(270,360)}
        front_scan = {**left_scan, **right_scan}
        if front_scan is None: front_scan = []
        poi = []
        output = (0,0)

        for theta in front_scan:
            r = front_scan[theta]
            theta_rad = theta * np.pi/180.0

            x = r * math.cos(theta_rad)
            y = r * math.sin(theta_rad)

            if -self.width < y < self.width and 0 < x < self.depth:
                poi.append((x,y))

        if poi:
            x_avg = [a[0] for a in poi]
            x_avg = sum(x_avg) / len(x_avg)

            y_avg = [a[1] for a in poi]
            y_avg = sum(y_avg) / len(y_avg)

            output = (x_avg, y_avg)

        return output

    def pub_marker(self, xy):
        x, y = xy
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
        
        marker.pose.position.x = x * 1.0
        marker.pose.position.y = y * 1.0

        self.marker_pub.publish(marker)

    def store_scan(self, msg):
        self.scan = {i:j for i,j in enumerate(msg.ranges) if j not in [0.0, np.inf]}

    def get_now(self):
        return self.get_clock().now().to_msg().sec


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()