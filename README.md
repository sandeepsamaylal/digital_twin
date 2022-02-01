Testing of the connection:
===============
ROS and hardware connection:

– Connect Ethernet from robot controller to PC and change IP address to 192.168.0.3. Also make sure IP address of robot is 192.168.0.100 
– On UR5 control pad, go to run program → under file tab click on load program → select nvidiaext.urp but not press run/start button 

Go to catkin_ws
```
catkin_make
source devel/setup.bash
```
– calibration 
```
roslaunch ur_calibration calibration_correction.launch   robot_ip:=192.168.0.100 target_filename:="${HOME}/catkin_ws/UR5_robot_calibration.yaml"
```
– close the script once calibration correction is done
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.100 [reverse_port:=REVERSE_PORT] kinematics_config:=${HOME}/catkin_ws/UR5_robot_calibration.yaml
```
– Press run/start on UR5 control pad
– Move to next step only when below message is seen on terminal window after running above script:
“Robot connected to reverse interface. Ready to receive control commands”

To test PC ↔ Robot interface connection to the following
``` 
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
```
– wait until you can start planning now message appears on terminal and open new window and launch below package:
```
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```
– test the working using motionplanning in rviz


– Testing simple_joint_control nvidia issac sdk example:
```
bazel run packages/universal_robots/apps:simple_joint_control
```
– TODO if not working, make sure status on robot control pad is stopped (if running click stop).
Then press run/start and wait for below message to appear on terminal window: 
“Robot connected to reverse interface. Ready to receive control commands.”
– Should be able to control the robot now with the slider GUI available on the juypter script.

– Testing simple_joint_control on simulation:
```
bazel run apps/samples/manipulation:simple_joint_control
```

Digital twin application:
===============
Simple Joint Control
```
bazel run packages/universal_robots/digital_twin_app:simple_joint_control
```
Pick and Place Application
```
bazel run packages/universal_robots/digital_twin_app:shuffle_box_hardware
```
