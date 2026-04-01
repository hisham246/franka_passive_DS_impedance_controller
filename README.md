# Passive-DS Impedance Controller
Implementation of the controller in the paper: K. Kronander and A. Billard, "Passive Interaction Control With Dynamical Systems" in *IEEE Robotics and Automation Letters*, vol. 1, no. 1, pp. 106-113, 2016.

This ROS package is built upon the `franka_ros` package (https://frankarobotics.github.io/docs/franka_ros/docs/index.html#), following the `franka_example_controllers` to structure the outline of all the components.

To install this package, please ensure that you have a catkin workspace present, and that `libfranka` and `franka_ros` are installed on your device. Please refer to the link above for instructions on how to do so. 

You can clone this repository into your `src` directory inside your catkin workspace, and then run `catkin_make` to compile it.

The main structure of the controller is written in the `src/passive_DS_impedance_controller.cpp` file with its header file in `include/passive_DS_impedance_controller.h`. 

To launch the controller on the robot, you can run the following command in your terminal:

`roslaunch franka_passive_ds_impedance_controller passive_DS_impedance_controller.launch robot_ip:=<your_robot_ip>`

This will activate gravity compensation on the robot by default before any velocity commands are being sent to the `/passiveDS/desired_twist` topic. This can be used to collect demonstrations to train a Dynamical System model. Once a message is published to the `/passiveDS/desired_twist` topic, the Passive-DS impedance controller automatically activates.

If you want to launch the controller inside the Gazebo simulation for the Franka Robotics Panda, make sure to paste the contents of the `franka_passive_ds_impedance_controller/config/passive_DS_impedance_controller.yaml` inside the `franka_ros/franka_gazebo/config/sim_controllers.yaml`, which would allow spawning the controllers with Gazebo. To launch the controller in gazebo, you can run the following command:

`roslaunch franka_gazebo panda.launch controller:=passive_DS_impedance_controller`

The scripts used for recording and commanding the end-effector velocities to the robot can be run in a separate terminal as follows after launching the controller:

**For recording the timestamped end-effector pose:** `rosrun franka_passive_ds_impedance_controller record_demos.py`

**For commanding the desired end-effector velocity:** `rosrun franka_passive_ds_impedance_controller command_robot.py`

## Prerequisites
This ROS package is built upon the `franka_ros` package (https://frankarobotics.github.io/docs/franka_ros/docs/index.html#), following the `franka_example_controllers` to structure the outline of all the components.

To install this package, please ensure that you have a catkin workspace present, and that `libfranka` and `franka_ros` are installed on your device. Please refer to the link above for instructions on how to do so. 

You can clone this repository into your `src` directory inside your catkin workspace, and then run `catkin_make` to compile it.

## Code Structure
The main structure of the controller is written in the `src/passive_DS_impedance_controller.cpp` file with its header file in `include/passive_DS_impedance_controller.h`. 

To launch the controller on the robot, you can run the following command in your terminal:

`roslaunch franka_passive_ds_impedance_controller passive_DS_impedance_controller.launch robot_ip:=<your_robot_ip>`

This will activate gravity compensation on the robot by default before any velocity commands are being sent to the `/passiveDS/desired_twist` topic. This can be used to collect demonstrations to train a Dynamical System model. Once a message is published to the `/passiveDS/desired_twist` topic, the Passive-DS impedance controller automatically activates.

If you want to launch the controller inside the Gazebo simulation for the Franka Robotics Panda, make sure to paste the contents of the `franka_passive_ds_impedance_controller/config/passive_DS_impedance_controller.yaml` inside the `franka_ros/franka_gazebo/config/sim_controllers.yaml`, which would allow spawning the controllers with Gazebo. To launch the controller in gazebo, you can run the following command:

`roslaunch franka_gazebo panda.launch controller:=passive_DS_impedance_controller`

The scripts used for recording and commanding the end-effector velocities to the robot can be run in a separate terminal as follows after launching the controller:

**For recording the timestamped end-effector pose:** `rosrun franka_passive_ds_impedance_controller record_demos.py`

**For commanding the desired end-effector velocity:** `rosrun franka_passive_ds_impedance_controller command_robot.py`