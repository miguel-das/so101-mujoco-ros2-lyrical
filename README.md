# SO101 with MoveIt

This ROS 2 workspace runs the SO101 arm in MuJoCo through `mujoco_ros2_control`, or on the real arm through `feetech_ros2_driver`. MoveIt plans motions and sends trajectories to the arm and gripper controllers.

## Packages

- `so101_description`: URDF, meshes, MuJoCo models, and a standalone model viewer.
- `so101_bringup`: MuJoCo or real hardware, `robot_state_publisher`, and ros2_control controllers.
- `so101_moveit_config`: MoveIt configuration and RViz planning interface.
- `so101_interfaces`: messages and actions, such as `MoveToPoint`, `MoveToJoints` and `MoveThroughWaypoints`.
- `so101_motion`: the `motion_server` MoveItPy node, which moves the arm to a Cartesian point, to joint positions or along Cartesian waypoints through actions.

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

### Motion actions

`so101_motion` starts the arm, RViz and the `motion_server` node from a single launch file. It uses MuJoCo by default; pass `hardware:=real` for the real arm (`usb_port` and `joint_config_file` work as in `hardware.launch.py`):

```bash
ros2 launch so101_motion motion.launch.py
ros2 launch so101_motion motion.launch.py hardware:=real
```

The RViz panel needs `rviz_visual_tools` (`sudo apt install ros-lyrical-rviz-visual-tools`). Send a gripper position as a `MoveToPoint` action goal:

```bash
ros2 action send_goal --feedback /motion_server/move_to_point so101_interfaces/action/MoveToPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.2, y: 0.0, z: 0.05}}}"
```

The node plans a move that reaches the point with the gripper pointing straight down (set `approach_pitch` in the goal to tilt it). Joint positions go to `MoveToJoints`: one value in rad per arm joint, in the order `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`:

```bash
ros2 action send_goal --feedback /motion_server/move_to_joints so101_interfaces/action/MoveToJoints \
  "{positions: [0.5, 0.0, 0.3, 0.0, 0.0]}"
```

To move only some joints, name them; the others keep their current position:

```bash
ros2 action send_goal --feedback /motion_server/move_to_joints so101_interfaces/action/MoveToJoints \
  "{joint_names: [shoulder_pan, elbow_flex], positions: [0.5, 0.3]}"
```

To follow a path, give `MoveThroughWaypoints` an n x 5 matrix, one `x, y, z, pitch, roll` row per waypoint (m and rad; `roll` is the `wrist_roll` joint angle). The arm reaches the first waypoint with a free path, then the gripper moves along straight lines through the others. `send_waypoints` reads the matrix from a CSV or YAML file:

```bash
ros2 run so101_motion send_waypoints install/so101_motion/share/so101_motion/config/waypoints_square.csv
```

The example traces a 5 cm square, 5 cm above the base. A YAML file holds a list of rows, either at the top level or under `waypoints:` next to an optional `frame_id:`. The goal fails before anything moves if a segment leaves the arm's reach, collides, or would make a joint jump (near a singularity). The `motion_server` parameters `cartesian_step` (0.005 m), `max_joint_step` (0.1 rad), `max_velocity_scaling_factor` and `max_acceleration_scaling_factor` (0.1) tune the path sampling and speed.

RViz replays each plan in a loop. Press **Next** in the RvizVisualToolsGui panel to execute it, or **Stop** to discard it. Feedback reports `PLANNING`, `WAITING_FOR_CONFIRMATION` and `EXECUTING`, and the result says whether the arm reached the goal.

A new goal on any action replaces a plan that hasn't been executed, and is rejected while the arm is planning or moving. **Stop** or canceling the goal (Ctrl+C in `ros2 action send_goal` or `send_waypoints`) also stops a running motion. Pass `confirm:=false` to execute every plan right away.

For a model viewer without the simulator or MoveIt, run `ros2 launch so101_description display.launch.py` instead.

## Real robot

The real arm uses [`feetech_ros2_driver`](https://github.com/ros-physical-ai/feetech_ros2_driver), included as a git submodule in `src/` (run `git submodule update --init` after cloning) and built with the rest of the workspace. Servo IDs must be 1 to 6, from `shoulder_pan` to `gripper`. On WSL2, first attach the USB serial adapter with `usbipd attach --wsl --busid <BUSID>`.

Calibrate the follower arm once with LeRobot before using it here: follow the [SO-101 calibration guide](https://huggingface.co/docs/lerobot/so101#calibrate). LeRobot stores each servo's homing offset and position limits in the servo's EEPROM, and the driver uses them as they are.

### Start the real robot

Start the hardware in one terminal, then MoveIt in another:

```bash
ros2 launch so101_bringup hardware.launch.py
ros2 launch so101_moveit_config moveit.launch.py use_sim_time:=false
```

`usb_port` defaults to the follower arm's `/dev/serial/by-id/` path; pass `usb_port:=/dev/ttyACM0` or your own board's path otherwise. Pass `joint_config_file:=/path/joints.yaml` to override per-joint servo settings such as PID gains.

Servo torque turns on when the hardware starts and turns off when it stops, so hold the arm before pressing Ctrl+C. Every joint must rest inside its URDF limit at startup: `arm_controller` clamps commands to those limits and would snap an out-of-range joint back into range. Check with `ros2 topic echo /joint_states --once` if unsure.

## Model and control

The URDF and MuJoCo scene are separate robot models. Keep joint names, axes, limits, and geometry aligned when changing either one. The active simulation model is `so101_description/mujoco/scene.xml`; `so101_camera_mount.xml` is an optional MuJoCo model with a wrist camera and is not selected by the launch file. ROS controllers are configured in `so101_bringup/config/controllers.yaml`, and MoveIt action mappings are in `so101_moveit_config/config/moveit_controllers.yaml`.
