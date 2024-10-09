#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Avoider:
    def __init__(self, vel_obj, obstacle_threshold=0.5, 
                 regional_angle=30, normal_lin_vel=0.5, 
                 trans_lin_vel=-0.09, trans_ang_vel=1.75):
        self.vel_obj = vel_obj
        self.OBSTACLE_DIST = obstacle_threshold
        self.REGIONAL_ANGLE = regional_angle
        self.NORMAL_LIN_VEL = normal_lin_vel
        self.TRANS_LIN_VEL = trans_lin_vel
        self.TRANS_ANG_VEL = trans_ang_vel
        
        # Initialize robot position
        self.position = [0.0, 0.0]
        self.orientation = 0.0

    Regions_Report = {
        "front_C": [], "front_L": [], "left_R": [], "left_C": [], "left_L": [],
        "back_R": [], "back_C": [], "back_L": [], "right_R": [], "right_C": [],
        "right_L": [], "front_R": []
    }

    Regions_Distances = {
        "front_C": 0, "front_L": 1, "left_R": 2, "left_C": 3, "left_L": 4,
        "back_R": 5, "back_C": 6, "back_L": -5, "right_R": -4, "right_C": -3,
        "right_L": -2, "front_R": -1
    }

    def update_position(self, odom):
        # Update position based on odometry data
        self.position[0] = odom.pose.pose.position.x
        self.position[1] = odom.pose.pose.position.y
        
        # Extract yaw (orientation) from quaternion
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        _, _, self.orientation = self.euler_from_quaternion(quaternion)

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def identify_regions(self, scan):
        REGIONS = [
            "front_C", "front_L", "left_R", "left_C", "left_L", 
            "back_R", "back_C", "back_L", "right_R", "right_C", 
            "right_L", "front_R"
        ]
        intermediary = scan.ranges[:int(self.REGIONAL_ANGLE / 2)] \
                     + scan.ranges[(len(scan.ranges) - 1) * int(self.REGIONAL_ANGLE / 2):]
        self.Regions_Report["front_C"] = [x for x in intermediary if x <= self.OBSTACLE_DIST and x != 'inf']

        for i, region in enumerate(REGIONS[1:]):
            self.Regions_Report[region] = [
                x for x in scan.ranges[self.REGIONAL_ANGLE * i:self.REGIONAL_ANGLE * (i + 1)] 
                if x <= self.OBSTACLE_DIST and x != 'inf'
            ]

    def avoid(self):
        act, ang_vel = self._clearance_test()
        self._steer(act, act * ang_vel)

        # Log the current velocity being sent to the robot
        rospy.loginfo(f"Commanding Velocity - Linear: {self.vel_obj.linear.x:.2f}, Angular: {self.vel_obj.angular.z:.2f}")

        # Log the robot's position and orientation
        rospy.loginfo(f"Robot Position: X: {self.position[0]:.2f}, Y: {self.position[1]:.2f}, Orientation: {math.degrees(self.orientation):.2f} degrees")

        return self.vel_obj

    def _clearance_test(self):
        goal = "front_C"
        closest = 10e6
        regional_dist = 0
        maxima = {"destination": "back_C", "distance": 10e-6}
        for region in self.Regions_Report.items():
            regional_dist = abs(self.Regions_Distances[region[0]] - self.Regions_Distances[goal])
            if not len(region[1]):
                if regional_dist < closest:
                    closest = regional_dist
                    maxima["distance"] = self.OBSTACLE_DIST
                    maxima["destination"] = region[0]
            elif max(region[1]) > maxima["distance"]:
                maxima["distance"] = max(region[1])
                maxima["destination"] = region[0]
        regional_dist = self.Regions_Distances[maxima["destination"]] - self.Regions_Distances[goal]
        return (closest != 0), (regional_dist / [abs(regional_dist) if regional_dist != 0 else 1][0]) * self.TRANS_ANG_VEL

    def _steer(self, steer=False, ang_vel=0):
        if not steer:
            self.vel_obj.linear.x = self.NORMAL_LIN_VEL
        else:
            self.vel_obj.linear.x = self.TRANS_LIN_VEL
        self.vel_obj.angular.z = ang_vel

def main():
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    vel_obj = Twist()
    avoider = Avoider(vel_obj)
    
    def scan_callback(scan):
        rospy.loginfo(f"Received LaserScan Data: {scan.ranges}")
        avoider.identify_regions(scan)
        cmd_vel = avoider.avoid()
        cmd_vel_pub.publish(cmd_vel)

    def odom_callback(odom):
        avoider.update_position(odom)  # This updates the robot's position
        # Log the received odometry data
        rospy.loginfo(f"Received Odometry - X: {odom.pose.pose.position.x:.2f}, Y: {odom.pose.pose.position.y:.2f}")

    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

