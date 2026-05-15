# Dobot CR10 Inverse Kinematics Project

This ROS 2 workspace demonstrates inverse kinematics and Cartesian trajectory
control for the Dobot CR10 robot in RViz.

The project uses a custom Python IK implementation and publishes directly to
`/joint_states`. It does not use MoveIt, MoveGroup, KDL, TRAC-IK, robot service
IK calls, or the Joint State Publisher GUI.

## What Is Included

- Custom CR10 forward kinematics
- Custom numerical Jacobian
- Custom damped least-squares inverse kinematics
- Straight-line end-effector motion from point A to point B
- Circular end-effector motion around center point A with radius R
- Reachability checks before motion starts
- RViz robot visualization
- RViz end-effector trace marker on `/ee_trace`

## Workspace Layout

```text
src/
  cr10_ik_project/
    cr10_ik_project/
      cr10_kinematics.py      # FK, Jacobian, IK, waypoint generation, reachability
      straight_line_ik.py     # straight-line motion node
      circular_ik.py          # circular motion node
    launch/
      straight_line_demo.launch.py
      circular_demo.launch.py
    rviz/
      cr10_demo.rviz

  DOBOT_6Axis_ROS2_V4/
    dobot_rviz/
      urdf/cr10_robot.urdf    # CR10 model used by robot_state_publisher
      meshes/                 # visual meshes for RViz
```

## Requirements

- ROS 2 Jazzy or compatible ROS 2 environment
- Python 3
- `colcon`
- `numpy`
- RViz 2

Install common ROS build tools if needed:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-numpy
```

Make sure ROS 2 is sourced before building:

```bash
source /opt/ros/jazzy/setup.bash
```

If your ROS distribution is not Jazzy, replace `jazzy` with your installed
distribution name.

## Build

From the workspace root:

```bash
cd ~/641_project
colcon build --packages-select cr10_ik_project
source install/setup.bash
```

## Run Straight-Line Motion

Default motion:

```bash
ros2 launch cr10_ik_project straight_line_demo.launch.py
```

Custom point A and point B:

```bash
ros2 launch cr10_ik_project straight_line_demo.launch.py \
  start_x:=-0.65 start_y:=0.35 start_z:=1.0 \
  end_x:=-0.65 end_y:=-0.35 end_z:=1.0 \
  steps:=200
```

The node will:

1. Generate Cartesian waypoints along the line from A to B.
2. Check that every waypoint is reachable.
3. If reachable, publish joint states to `/joint_states`.
4. Publish the end-effector trace to `/ee_trace`.

## Run Circular Motion

Default motion:

```bash
ros2 launch cr10_ik_project circular_demo.launch.py
```

Custom center point A and radius R:

```bash
ros2 launch cr10_ik_project circular_demo.launch.py \
  center_x:=-0.55 center_y:=0.0 center_z:=0.9 \
  radius:=0.4 \
  steps:=300
```

The circle is parallel to the ground, so the Z height stays constant.

## Unreachable Input Examples

Straight-line unreachable example:

```bash
ros2 launch cr10_ik_project straight_line_demo.launch.py \
  start_x:=10.0 start_y:=10.0 start_z:=10.0 \
  end_x:=10.1 end_y:=10.0 end_z:=10.0
```

Circular unreachable example:

```bash
ros2 launch cr10_ik_project circular_demo.launch.py \
  center_x:=10.0 center_y:=10.0 center_z:=10.0 \
  radius:=0.4
```

If the trajectory is unreachable, the node prints an error and does not start
motion.

## Callable Motion Functions

The trajectory functions are callable from Python code:

```python
from cr10_ik_project.straight_line_ik import StraightLineIK
from cr10_ik_project.circular_ik import CircularIK

line_node = StraightLineIK()
line_node.move_line([-0.65, 0.35, 1.0], [-0.65, -0.35, 1.0], steps=200)

circle_node = CircularIK()
circle_node.move_circle([-0.55, 0.0, 0.9], 0.4, steps=300)
```

Both methods return:

- `True` if the full trajectory is reachable and motion is enabled.
- `False` if any waypoint is unreachable.

## Test

```bash
cd ~/641_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select cr10_ik_project
```

## Notes

- Use the launch files in `cr10_ik_project`, not the vendor MoveIt launch files.
- The Dobot vendor packages are included only for robot model/URDF/mesh support.
- The custom IK logic is in `cr10_ik_project/cr10_kinematics.py`.
