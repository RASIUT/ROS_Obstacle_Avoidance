Create a catkin workspace using 
mkdir -p ~/catkin_ws/src

then create a directory named turtlebot3 using
mkdir turtlebot3

now create the catkin package named turtlebot3_example using 
catkin_create_pkg turtlebot_example rospy std_msgs roscpp

then inside this package paste the scripts file and get back to turtlebot3 directory using
cd ~/catkin_ws/src/turtlebot3

then use catkin_make
now source the file using
source devel/setup.bash

use chmod +x src/turtlebot3/turtlebot3_example/scripts/laser_obstacle_avoid_360_node_class.py
then go to catkin_ws and roslaunch turtlebot3_gazebo turtlebot3_world.launch

then rosrun turtlebot3_example scripts/laser_obstacle_avoid_360_node_class.py
