import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

velocity_of_car = 3
L = 0.5  # Lookahead distance
wall_margin = 0.5
gain = 2
scan_sample_rate = 5  # Process every 5th scan point to reduce computation


class ScannerToCmdVelController:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.scanner_sub = rospy.Subscriber('/scanner1', LaserScan, self.scanner_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_data = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scanner_callback(self, data):
        self.scan_data = data.ranges[::scan_sample_rate]  # Process fewer points
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment

    def run(self):
        waypoints = [(-2, -4), (4, 4), (12, 0)]
        waypoint_counter = 0
        rate = rospy.Rate(10)  # Reduce frequency to lower CPU load
        
        while not rospy.is_shutdown():
            if self.scan_data is None:
                continue  # Skip iteration if no data

            waypoint = waypoints[waypoint_counter]
            dx = waypoint[0] - self.current_x
            dy = waypoint[1] - self.current_y
            distance = math.sqrt(dx**2 + dy**2)

            # Optimized min distance search
            min_distance = min(self.scan_data) if self.scan_data else float('inf')
            min_distance_index = self.scan_data.index(min_distance) if self.scan_data else -1
            min_distance_location = "left" if min_distance_index < len(self.scan_data) // 2 else "right"

            if min_distance < wall_margin:
                linear_x = 0
                angular_z = 100 if min_distance_location == "right" else -100
            else:
                if distance < L:
                    rospy.loginfo(f"{waypoint} reached!")
                    waypoint_counter = (waypoint_counter + 1) % len(waypoints)
                    linear_x, angular_z = velocity_of_car, 0
                else:
                    gamma_angle = math.atan2(dy, dx)
                    steering_angle = math.atan2(math.sin(gamma_angle - self.yaw), math.cos(gamma_angle - self.yaw))
                    linear_x = velocity_of_car
                    angular_z = steering_angle * gain

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_x
            cmd_vel_msg.angular.z = angular_z
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = ScannerToCmdVelController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
