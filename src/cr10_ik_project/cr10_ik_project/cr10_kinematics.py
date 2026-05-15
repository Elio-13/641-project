import math

import numpy as np


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
JOINT_LOWER_LIMITS = np.array([-6.27, -6.27, -2.861, -6.27, -6.27, -6.27])
JOINT_UPPER_LIMITS = np.array([6.27, 6.27, 2.861, 6.27, 6.27, 6.27])


def trans(x, y, z):
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1],
    ])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1],
    ])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def rpy(r, p, y):
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def forward_transform(q):
    T = np.eye(4)

    T = T @ trans(0, 0, 0.1765) @ rpy(0, 0, 0) @ rot_z(q[0])
    T = T @ trans(0, 0, 0) @ rpy(1.5708, 1.5708, 0) @ rot_z(q[1])
    T = T @ trans(-0.607, 0, 0) @ rpy(0, 0, 0) @ rot_z(q[2])
    T = T @ trans(-0.568, 0, 0.191) @ rpy(0, 0, -1.5708) @ rot_z(q[3])
    T = T @ trans(0, -0.125, 0) @ rpy(1.5708, 0, 0) @ rot_z(q[4])
    T = T @ trans(0, 0.1084, 0) @ rpy(-1.5708, 0, 0) @ rot_z(q[5])

    return T


def forward_position(q):
    return forward_transform(q)[0:3, 3]


def numerical_jacobian(q, eps=1e-5):
    J = np.zeros((3, 6))
    p0 = forward_position(q)

    for j in range(6):
        dq = np.zeros(6)
        dq[j] = eps
        p1 = forward_position(q + dq)
        J[:, j] = (p1 - p0) / eps

    return J


def solve_position_ik(
    target,
    q_seed,
    max_iterations=80,
    tolerance=0.002,
    step_size=0.45,
    damping=0.08,
):
    q = np.asarray(q_seed, dtype=float).copy()
    target = np.asarray(target, dtype=float)

    for _ in range(max_iterations):
        current = forward_position(q)
        error = target - current

        if np.linalg.norm(error) < tolerance:
            break

        J = numerical_jacobian(q)
        lhs = J @ J.T + (damping ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(lhs, error)

        q = q + step_size * dq
        q = np.clip(q, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

    return q


def straight_line_waypoints(start, end, steps):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)

    for i in range(steps + 1):
        s = i / steps
        yield (1.0 - s) * start + s * end


def circular_waypoint(center, radius, step, steps):
    center = np.asarray(center, dtype=float)
    theta = 2.0 * math.pi * step / steps

    return np.array([
        center[0] + radius * math.cos(theta),
        center[1] + radius * math.sin(theta),
        center[2],
    ])
