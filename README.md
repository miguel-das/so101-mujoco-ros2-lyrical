# SO101 simulation with MoveIt

This ROS 2 workspace runs the SO101 arm in MuJoCo through `mujoco_ros2_control`. MoveIt plans motions and sends trajectories to the simulated arm and gripper controllers.

## Packages

- `so101_description`: URDF, meshes, MuJoCo models, and a standalone model viewer.
- `so101_bringup`: MuJoCo, `robot_state_publisher`, and ros2_control controllers.
- `so101_moveit_config`: MoveIt configuration and RViz planning interface.

## Setup

Use ROS 2 Lyrical with a graphical desktop. Install dependencies declared by the packages, including `mujoco_ros2_control`, MoveIt, and the ros2_control trajectory controllers:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the simulator in one terminal:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch so101_bringup mujoco.launch.py
```

Wait for `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` to activate. Then start MoveIt in another terminal:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch so101_moveit_config moveit.launch.py
```

In RViz, use the **MotionPlanning** panel. Select the `arm` group, set a joint or pose goal, then choose **Plan & Execute**. Select the `gripper` group to command the jaw; its named `open` and `closed` states are available as goals. MoveIt uses the OMPL planner and the simulated `FollowJointTrajectory` actions. The initial arm pose is the `home` state.

For a model viewer without the simulator or MoveIt, run `ros2 launch so101_description display.launch.py` instead.

## Model and control

The URDF and MuJoCo scene are separate robot models. Keep joint names, axes, limits, and geometry aligned when changing either one. The active simulation model is `so101_description/mujoco/scene.xml`; `so101_camera_mount.xml` is an optional MuJoCo model with a wrist camera and is not selected by the launch file. ROS controllers are configured in `so101_bringup/config/controllers.yaml`, and MoveIt action mappings are in `so101_moveit_config/config/moveit_controllers.yaml`.
