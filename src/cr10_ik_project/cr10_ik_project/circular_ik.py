from cr10_ik_project.cr10_kinematics import (
    circular_waypoint,
    forward_position,
    JOINT_NAMES,
    solve_position_ik,
)
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class CircularIK(Node):
    def __init__(self):
        super().__init__('circular_ik')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/ee_trace', 10)

        self.declare_parameter('center', [-0.55, 0.0, 0.90])
        self.declare_parameter('radius', 0.4)
        self.declare_parameter('steps', 300)

        self.center = np.array(self.get_parameter('center').value, dtype=float)
        self.radius = self.get_parameter('radius').value
        self.steps = self.get_parameter('steps').value
        self.i = 0
        self.q = np.array([0.0, 0.4, -0.4, 0.0, 0.0, 0.0])

        self.trace_points = []
        self.timer = self.create_timer(0.03, self.update)

    def solve_ik(self, target):
        self.q = solve_position_ik(target, self.q)
        return self.q

    def publish_joints(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = q.tolist()
        self.joint_pub.publish(msg)

    def current_ee_point(self, q):
        ee = forward_position(q)
        p = Point()
        p.x = float(ee[0])
        p.y = float(ee[1])
        p.z = float(ee[2])
        return p

    def publish_trace(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'circular_trace'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = self.trace_points
        self.marker_pub.publish(marker)

    def update(self):
        target = circular_waypoint(self.center, self.radius, self.i, self.steps)

        q = self.solve_ik(target)
        self.publish_joints(q)
        self.trace_points.append(self.current_ee_point(q))

        self.publish_trace()

        self.i += 1
        if self.i > self.steps:
            self.i = 0
            self.trace_points = []


def main(args=None):
    rclpy.init(args=args)
    node = CircularIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
