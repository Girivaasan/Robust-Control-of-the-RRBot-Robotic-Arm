# Robust-Control-of-the-RRBot-Robotic-Arm

## Creating a ROS Workspace

**Follow these commands**
```
mkdir -p ~/RRbot_ros/src
cd ~/RRbot_ros
catkin_make
echo "source ~/RRbot_ros/devel/setup.bash" >> ~/.bashrc
source ~/RRbot_ros/devel/setup.bash
```
## Clone the repository
```
cd ~/RRbot_ros/src
git clone https://github.com/ros-simulation/gazebo_ros_demos.git
```
## Update the workspace
```
sudo apt-get update
sudo apt-get upgrade
sudo apt update
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```
## Catkin-make
```
cd ~/RRbot_ros
catkin_make
```
## Launching the robot in Gazebo
```
roslaunch rrbot_gazebo rrbot_world.launch
```
## Running the control node
```
roslaunch rrbot_control rrbot_effort_control.launch
```
**After running the control node, run the 'rrbot_robust_control.m' matlab file provided in the repository.**

https://user-images.githubusercontent.com/118299474/209412574-bb3b6c8b-1dce-431b-9b49-c7b86aa4136a.mp4

## Performance Plots
The plot depicts the desired trajectory generated, that must be followed by the rrbot manipulator and the performance of the manipulator. 

![Screenshot from 2022-11-29 11-16-52](https://user-images.githubusercontent.com/118299474/209414791-3d249be6-3abe-48da-b595-69fc27e8b2a0.png)

After implementing robust-control, theshattering issue in the torques is removed.

![Screenshot from 2022-11-28 15-48-25](https://user-images.githubusercontent.com/118299474/209414859-e867756f-3ff4-4083-9c28-c5c5fd58c12d.png)

