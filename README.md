# Turtlebot follow a Red turtlebot using RGB-D camera

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
  - Image Processing Toolbox

<i>The model used in this project is TURTLEBOT3 WAFFLE, this model have a RGB-D camera built in can can be set by: </i>
<code> export TURTLEBOT3_MODEL=waffle </code>

## Setup and Launching the simulation
Replace the current, <code>turtlebot3_waffle.gazebo.xacro </code>, <code>turtlebot3_waffle.urdf.xacro</code> files with:
- <code>turtlebot3_waffle_red.gazebo.xacro</code>
- <code>turtlebot3_waffle_red.urdf.xacro</code>

Add the world and launch file to your gazebo world and launch file directories.
Before launch, the files have to be sourced:
<code>source devel/setup.bash</code>

The simulation can be launched by:
<code>roslaunch turtlebot3_gazebo turtlebot3_warehouse_withQR.launch</code>
<h2><b> Operation </b></h2> 
The robot 


## Code Structure
<h3><b>FollowRedTurtlebot.m Class</b></h3>
The [`FollowRedTurtlebot.m`] script defines a MATLAB class that enables the robot to follow a red object (in this case, another turtlebot) using its camera. The robot processes the camera images to detect the red object, calculates its relative position and distance, and then determines the appropriate motion commands to follow the object at a given trailing distance.
<br>
<br>
<b><i>Methods</b></i>
<br>

<b>Constructor:</b>
- Initializes the ROS environment, sets up the ROS node, subscribers, and publishers.
- Continuously processes the camera images to determine the robot's actions.

<b>ProcessImage:</b>
- Retrieves the RGB image and extracts the red, green, and blue channels.
- Computes the weighted centroid of the detected red regions.
- Labels the red regions and filters out regions below a minimum area threshold.
- Retrieves the depth image and calculates the relative angle to the detected red object.
- Determines the robot's motion based on the detected red object's position and depth.

<b>SearchRobot:</b>
- If no red object is detected, the robot spins in place.

<b>TrackRobot:</b>
- Determines the robot's motion commands based on the estimated distance to the detected red object, the rotation speed, the angle error, and the depth value.

<b>HoldRobot:</b>
- Stops the robot's motion.

<h3><b>detectRed.m Script</b></h3>
The [`detectRed.m`] script was the initial implementation of detecting red and always used the perceived size of the object to evaluate the trailing distance. Not used in the final implementation.



## Contribution
Lukas Kiehl: *34%*

Nicholas Keltchine: *33%*

Alexander Tweed: *33%*
