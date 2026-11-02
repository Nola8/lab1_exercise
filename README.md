# ROS 2 Robotics Lab: Autonomous Navigation & Sensor Integration

This repository contains the code, launch files, and interface packages developed for the ROS 2 (Humble) robotics lab. It implements differential drive robot control, custom action servers (`/drive_distance` and `/rotate`), autonomous square path tracking, obstacle detection, and sensor bridging (LiDAR, IMU, Depth Camera).

---

## 1. Build Instructions

To build and compile all packages in your workspace (`ros2_ws`), open a terminal in your workspace root and run:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
## Step 1

Run the gazebo and rviz display to see the depth camera
```bash
ros2 launch diffdrive_bot gazebo.launch.py
```
we can see the imu listed in the TF tree.

<img width="206" height="140" alt="image" src="https://github.com/user-attachments/assets/fb5e0533-9a40-4ffb-9fce-d4ea52537e3d" />

screenshot of the depth camera when back is facing the cube box.
we can also run the following instructions for more information. (After the gazebo is launched.)
```bash
ros2 run tf2_ros tf2_echo base_link depth_camera_link
ros2 topic hz /imu
```

## Step 2-5
Start the simulation environment in your first terminal:

```Bash
ros2 launch diffdrive_bot control.launch.py
```
Run your new node in a second terminal:
```Bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_nodes turn_away
```
Testing in a third terminal:
```Bash
#check params for step3
ros2 param list /turn_away
ros2 param set /turn_away forward_speed 0.4

#check robot_status for step 4
ros2 interface show my_robot_interfaces/msg/RobotStatus
ros2 topic echo /robot_status

#check service for step 5
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"

#valid Value 
ros2 service call /set_turn_speed my_robot_interface/srv/SetTurnSpeed "{turn_speed: 1.0}"

# invalid value above the max
ros2 service call /set_turn_speed my_robot_interface/srv/SetTurnSpeed "{turn_speed: 3.0}"
```
Screenshot of RQT Graph

<img width="586" height="253" alt="image" src="https://github.com/user-attachments/assets/a204c492-6b42-416b-aba7-97a8c96ef694" />

## Step 6
Run the simulation environment in Terminal 1:

```Bash
ros2 launch diffdrive_bot control.launch.py
```
Run your action server in Terminal 2:
```Bash
ros2 run my_robot_nodes rotate_server
```
Send a test goal with feedback enabled in Terminal 3:
```Bash
ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.57}"
```

## Step 7
```Bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
```
### What didn't work?
The when the robot drive in square, the original angle is not preserved after the square is formed. I think the reason for this the angle of the Rotate action meaning when it turns the rate might be high so the accuracy is a bit low so these little inaccuracies add up. 




