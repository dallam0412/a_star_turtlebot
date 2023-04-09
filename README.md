# A-star ROS implementation

## Description
We have implemented A-star algorithm for a robot with non-holonomic constraints. First, it was visualized using pygame, and the same was implemented in gazebo using ROS

## Authors
1. Dhanush Babu Allam - 119390320
2. Sourang Sri Hari - 119074947

## Dependencies
- Python3
- ROS noetic
- Gazebo
- turtlebot3
- numpy
- pygame
- matplotlib
- queue
- sortedcollections
- pygame
- rospy

## Github Repo link
```
https://github.com/dallam0412/a_star_turtlebot
```

## Cloning Repo

Clone the repo in a convinent location
```
git clone https://github.com/dallam0412/a_star_turtlebot.git
```
Create catkin_ws

```
mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
```
copy and paste the proj_ph_2, available in Part02 into the src of the catkin_ws
 <br>Additionally make sure turtlebot_simulations is also in the src folder

## Building the package

Make sure your are in the workspace folder

```
caktin_make
```

Source the folder
```
source devel/setup.bash
```

## Part1 - A-star (Pygame visualization)
For running part1 of phase2, go to the folder you cloned, open the Part01 folder and run the following command
```
python3 a_star_dhanush_sourang.py
```
### Sample inputs for running part1

enter the clearance in mm : 100 </br>
input start x coordinate: 0.5 </br>
input start y coordinate: 1 </br>
input start orientation: 0 </br>
input goal x coordinate: 2 </br>
input goal y coordinate: 1.7 </br>
Enter RPM1 : 80 </br>
Enter RPM2 : 100 </br>

## Part2 - ROS (Gazebo Visualization)
For running part2 of phase2 i.e., Gazebo Visualization, run the following command (make sure you have sourced ros and the workspace in the terminal you are using)
```
roslaunch proj_ph_2 A_star.launch clearance:=100 cord_x:=0.0 cord_y:=0.0 initial_theta:=0 goal_x_coord:=5 goal_y_coord:=0 rpm1:=55 rpm2:=50
```
# Note
only clearance is in mm, x_coord, y_coord for both goal and start are in meters. The orientation is in degrees.

## Output
### Video link for Phase2 Part1
```
https://drive.google.com/file/d/1hqxUBLnsQtDSFpFt_nMNAspz0HFXIWex/view?usp=share_link
```
### Video link for Phase2 Part2 
```
https://drive.google.com/file/d/1q_xxL8HfCcxpVKNZtf3_ciWqOS0fhtSI/view?usp=share_link
```
