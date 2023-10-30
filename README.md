# Turtlebot follow a square using RGB-D camera and QR code

This project use MATLAB and ROS environment to control a turtlebot 3 waffle. By using a RGB-D camera, the robot is capable of tracking a 

<b> <i>Required Software: </i></b>
- Ubuntu 18.04
- MATLAB 2023a
- ROS melodic

<b> <i>Packages for ROS: </i></b>
  - turtlebot3_simulation -  https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  - turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3.git

<b> <i>Packages for MATLAB: </i></b>
  - ROS Toolbox

<i>The model used in this project is TURTLEBOT3 WAFFLE, this model have a RGB-D camera built in can can be set by: </i>
<code> export TURTLEBOT3_MODEL=waffle </code>

<h2><b>Setup and Launching the simulation</b></h2>
Copy the files in the turtlebot3_gazebos.zip into the path: catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo
Before launch,the files have to be sourced:
<code>source devel/setup.bash</code>

The simulation can be launched by:
<code>roslaunch turtlebot3_gazebo turtlebot3_warehouse_withQR.launch</code>
<h2><b> Operation </b></h2> 
The robot 



## Code Structure

### Contribution
Lukas Kiehl: *34%*

Nicholas Keltchine: *33%*

Alexander Tweed: *33%*
