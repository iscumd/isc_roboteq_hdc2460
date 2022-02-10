# Roboteq Controller

## Summary

This ROS2 wrapper for the Roboteq HDC2460 and the Roboteq AX2850 controllers. The node searches all open 
usb ports and finds the one the roboteq is assigned to. from there it computes the left and right wheel speeds, multiplies it by 
the speed multiplier, which is 60 in our case  and then write it over RS -232 to the controller.

## Topics 

### Publisher

- no topics are pubished from this node

### Subscriber

- this node subscribes to the `/cmd_vel` topic published from nav2 or teleop

## Params 

This node has 3 parameters
 
 1. Max current
 2. Baudrate 
 3. chunck size

## Launch 

To launch this package, make sure the roboteq controller is plugged into a usb port and is powered on, after that, 
source ros and the work space and then type the command `ros2 launch isc_roboteq isc_roboteq.launch.py`

## install/Build
1. Change directory into your colcon workspace folder. Example: `cd ~/ros_ws/`
2. Clone the repository into your colcon workspace: `vcs import src --input https://raw.githubusercontent.com/girvenavery2022/isc_roboteq_hdc2460/ros2/roboteq.repos`
3. Build the colcon workspace: `colcon build`
4. Source the local workspace: `. install/setup.bash`
