from cr10_ik_project.cr10_kinematics import (
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


class StraightLineIK(Node):
    def __init__(self):
        super().__init__('straight_line_ik')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/ee_trace', 10)

        self.declare_parameter('start', [-0.65, 0.35, 1.00])
        self.declare_parameter('end', [-0.65, -0.35, 1.00])
        self.declare_parameter('steps', 200)

        self.start = np.array(self.get_parameter('start').value, dtype=float)
        self.end = np.array(self.get_parameter('end').value, dtype=float)
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
        marker.ns = 'straight_line_trace'
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
        s = self.i / self.steps
        target = (1.0 - s) * self.start + s * self.end

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
    node = StraightLineIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
