**<ins>TERMINAL 0 :</ins>**

- mkdir ~/catkin_ws/src

- cd ~/catkin_ws/src

- mkdir turtlebot3

- cd turtlebot3

- catkin_create_pkg turtlebot3_example roscpp rospy std_msgs sensor_msgs

**NOW PASTE THE** "*scripts*" **FILE INTO THE PACKAGE**

- cd ~/catkin_ws/src

- catkin_make

- source devel/setup.bash

- chmod +x src/turtlebot3/turtlebot3_example/scripts/laser_obstacle_avoid_360_node_class.py


**<ins>TERMINAL 1 :</ins>** roscore

**<ins>TERMNAL 2 :</ins>** 

  - catkin_make
  
  - source devel/setup.bash
  
  - roslaunch turtlebot3_gazebo turtlebot3_world.launch

**<ins>TERMINAL 0 :</ins>** 

  - rosrun turtlebot3_example/scripts/laser_obstacle_avoid_360_node_class.py
