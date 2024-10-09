#!/usr/bin/env python3

from Avoider import Avoider
import rospy
from geometry_msgs.msg import Twist  # ROS msg that deals with moving the robot
from sensor_msgs.msg import LaserScan  # ROS msg that gets the laser scans

def main():
    vel = Twist()
    # Instantiate our avoider object
    avoider = Avoider(vel)
    # Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    # Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, avoider.identify_regions)
    # Create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # ROS will try to run this code 10 times/second
    rate = rospy.Rate(10)  # 10Hz
    
    # Keep running while the ROS master isn't shutdown
    while not rospy.is_shutdown():
        vel = avoider.avoid()
        pub.publish(vel)

        # Removed the call to the dummy matrix function
        # Add any other logic if needed for current_position here

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

