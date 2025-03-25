import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Add this import
import math
import numpy as np

velocity_of_car = 3
L = 0.3
wall_margin = 0.1
radius_of_car = 0.5
distance_threshold = 1
angle_weight = 10
gap_width_weight = 1
gap_distance_weight = 1
gain = 1.5

class ScannerToCmdVelController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('controller', anonymous=True)

        # Created a subscriber for the scanner1 topic
        self.scanner_sub = rospy.Subscriber('/scanner1', LaserScan, self.scanner_callback)

        # Created a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Variable to hold the latest scan data
        self.scan_data = None

        # Add these lines for odometry -------------------------------
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_x = 0.0  # X coordinate will be updated here
        self.current_y = 0.0  # Y coordinate will be updated here
        self.yaw = 0.0
        # ------------------------------------------------------------

    def odom_callback(self, msg):
        # Simple odometry data extraction (add this method)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scanner_callback(self, data):
        self.scan_data = data.ranges  # Store distance values
        self.angle_min = data.angle_min # Minimum scan angle
        self.angle_max = data.angle_max  # Maximum scan angle
        self.angle_increment = data.angle_increment  # Angle step size
        
    def run(self):
        waypoints = [(-2,-4),(4,4),(12,0)] # these are the waypoints to be followed for the first task
        waypoint_counter = 0
        rate = rospy.Rate(30)
        omit_counter = 0
        while not rospy.is_shutdown():

            if self.scan_data is None:
                rospy.logwarn("Waiting for scan data...")
                rate.sleep()
                omit_counter += 1
                rospy.loginfo(f"omit_counter : {omit_counter}")
                continue
                    
            waypoint = waypoints[waypoint_counter]
            x = self.current_x
            y = self.current_y  
            yaw = self.yaw

            x_waypoint = waypoint[0]
            y_waypoint = waypoint[1]

            dx = x_waypoint - x
            dy = y_waypoint - y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < L:
                rospy.loginfo(f"Reached waypoint: {waypoint}")
                waypoint_counter = (waypoint_counter + 1) % len(waypoints)
                continue

            number_of_readings = len(self.scan_data)

            lidar_ranges = np.zeros(number_of_readings)

            j=0
            for i in range(number_of_readings//2, 0, -1):
                lidar_ranges[j] = self.scan_data[i]
                j+=1
            for i in range(number_of_readings-1, number_of_readings//2, -1):
                lidar_ranges[j] = self.scan_data[i]
                j+=1

            lidar_ranges = np.clip(lidar_ranges, 0, distance) # Radius of consideration concept

            min_dist = float('inf')
            min_dist_index = None
            for i in range(number_of_readings):
                if lidar_ranges[i] < min_dist:
                    min_dist = lidar_ranges[i]
                    min_dist_index = i

            theta = math.atan2(radius_of_car ,min_dist)
            n = int(((2*theta)/(math.pi*2))*(number_of_readings))
            
            if min_dist_index < n//2:
                min_dist_index = n//2
            if min_dist_index+n//2 > number_of_readings:
                min_dist_index = number_of_readings - n//2
            for i in range(min_dist_index - n//2, min_dist_index+n//2, 1):
                lidar_ranges[i] = 0
                        
            gaps = []
            number_of_gaps = 0
            start = end = None
            for i in range(number_of_readings-1):
                if start is None:
                    start = i
                if (abs(lidar_ranges[i] - lidar_ranges[i+1]) > distance_threshold) or (i==number_of_readings-2):
                    end = i
                    gaps.append([start, end])
                    number_of_gaps += 1
                    start = end = None
            
            max_potential = 0
            max_potential_angleCG = None

            angleCW = math.atan2(dy, dx) - yaw # Angle between Car and Waypoint
            angleCW = math.atan2(math.sin(angleCW), math.cos(angleCW))

            for i in range(number_of_gaps):
                start = gaps[i][0]
                end = gaps[i][1]
                mid = (start+end)//2
                angleCG = math.pi - (mid/number_of_readings)*(2*math.pi) #Angle between Car and Gap Midpoint
                angleGW = abs(angleCG - angleCW) # Angle between Gap and Waypoint
                angleGW = min(angleGW, 2*math.pi - angleGW)
                gap_width = end-start
                gap_distance = lidar_ranges[mid]
                potential = (gap_width**gap_width_weight)*(gap_distance**gap_distance_weight)*(1/(angle_weight*angleGW+1))
                if potential > max_potential:
                    max_potential = potential
                    max_potential_angleCG = angleCG
    
                
            if max_potential_angleCG == None:
                steering_angle = angleCW
            else:
                steering_angle = max_potential_angleCG
            steering_angle = steering_angle*gain

            

            cmd_vel_msg = Twist()
            if min_dist < wall_margin:
                cmd_vel_msg.linear.x = -velocity_of_car
                rospy.loginfo(f"Car has crashed :(")
            else:
                cmd_vel_msg.linear.x = velocity_of_car
            cmd_vel_msg.angular.z = steering_angle
            self.cmd_vel_pub.publish(cmd_vel_msg)
            
            rate.sleep()

if __name__ == '__main__':
    print("Starting Controller")
    try:
        controller = ScannerToCmdVelController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller terminated.")
