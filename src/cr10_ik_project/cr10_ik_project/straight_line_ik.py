from cr10_ik_project.cr10_kinematics import (
    DEFAULT_Q_SEED,
    forward_position,
    JOINT_NAMES,
    validate_straight_line,
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

        self.declare_parameter('start_x', -0.65)
        self.declare_parameter('start_y', 0.35)
        self.declare_parameter('start_z', 1.00)
        self.declare_parameter('end_x', -0.65)
        self.declare_parameter('end_y', -0.35)
        self.declare_parameter('end_z', 1.00)
        self.declare_parameter('steps', 200)

        self.steps = self.get_parameter('steps').value
        self.i = 0
        self.q = DEFAULT_Q_SEED.copy()
        self.joint_trajectory = []
        self.motion_enabled = False

        self.trace_points = []
        start = [
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value,
            self.get_parameter('start_z').value,
        ]
        end = [
            self.get_parameter('end_x').value,
            self.get_parameter('end_y').value,
            self.get_parameter('end_z').value,
        ]
        self.move_line(start, end, self.steps)

        self.timer = self.create_timer(0.03, self.update)

    def move_line(self, point_a, point_b, steps=200):
        point_a = np.array(point_a, dtype=float)
        point_b = np.array(point_b, dtype=float)
        self.steps = steps

        reachable, joint_trajectory, message = validate_straight_line(
            point_a,
            point_b,
            self.steps,
            self.q,
        )

        if not reachable:
            self.motion_enabled = False
            self.joint_trajectory = []
            self.trace_points = []
            self.get_logger().error(
                'Straight-line motion not started: {}'.format(message)
            )
            return False

        self.i = 0
        self.q = joint_trajectory[0].copy()
        self.joint_trajectory = joint_trajectory
        self.trace_points = []
        self.motion_enabled = True
        self.get_logger().info(
            'Straight-line motion accepted from A={} to B={}.'.format(
                point_a.tolist(),
                point_b.tolist(),
            )
        )
        return True

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
        if not self.motion_enabled:
            return

        q = self.joint_trajectory[self.i]
        self.q = q.copy()
        self.publish_joints(q)
        self.trace_points.append(self.current_ee_point(q))

        self.publish_trace()

        self.i += 1
        if self.i >= len(self.joint_trajectory):
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
