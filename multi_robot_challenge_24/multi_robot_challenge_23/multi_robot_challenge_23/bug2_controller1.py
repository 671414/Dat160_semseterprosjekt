import math
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from multi_robot_interfaces.srv import Gotopoint
from std_srvs.srv import SetBool
from tf_transformations import euler_from_quaternion


class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')

        # Initialize service clients for wall follower and go-to-point
        self.wall_follower_client = self.create_client(SetBool, '/wallfollower')
        self.wall_follower_client.wait_for_service()

        self.go_to_point_client = self.create_client(Gotopoint, '/gotopoint')
        self.go_to_point_client.wait_for_service()

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Point, '/big_fire_position', self.big_fire_callback, 10)  # Listen for target position

        # Set up initial variables
        self.target_position = None  # Target position for Bug2 navigation
        self.current_position = Point()
        self.is_wall_following = False
        self.leave_point = None
        self.obstacle_detected = False
        self.is_active = False  # Bug2 behavior inactive until target is received

        # Timer for periodic checks
        self.wall_following_start_time = None
        self.min_wall_follow_duration = timedelta(seconds=5)  # Stay in wall-following for 5 seconds
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Bug2 Controller initialized. Waiting for big fire detection.")

    def laser_callback(self, msg):
        # Processes laser scan data to detect obstacles
        self.front = min(msg.ranges[0:20] + msg.ranges[340:360])
        self.obstacle_detected = self.front < 1.0

    def odom_callback(self, msg):
        # Updates the robot's current position and yaw from odometry
        position = msg.pose.pose.position
        self.current_position.x = position.x
        self.current_position.y = position.y
        self.current_position.z = position.z

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def big_fire_callback(self, msg):
        # Receives the target position for Bug2 navigation
        self.target_position = msg
        self.is_active = True
        self.get_logger().info(f"Big Fire detected! Target position set: x={msg.x}, y={msg.y}. Activating Bug2 behavior.")

    def timer_callback(self):
        # Periodically checks and manages Bug2 behavior
        if not self.is_active or self.target_position is None:
            return

        if self.obstacle_detected:
            if not self.is_wall_following:
                self.start_wall_following()
            elif self.can_leave_wall():
                self.get_logger().info("Obstacle cleared, returning to goal.")
                self.stop_wall_following()
        else:
            self.go_to_goal()

    def start_wall_following(self):
        # Initiates wall-following behavior when an obstacle is detected
        self.is_wall_following = True
        self.leave_point = self.current_position
        self.wall_following_start_time = datetime.now()
        self.get_logger().info("Starting wall-following behavior.")

        # Activate wall-following mode
        request = SetBool.Request()
        request.data = True
        self.wall_follower_client.call_async(request)

    def stop_wall_following(self):
        # Stops wall-following behavior and resumes navigation to the goal
        self.is_wall_following = False
        self.get_logger().info("Stopping wall-following behavior.")

        # Deactivate wall-following mode
        request = SetBool.Request()
        request.data = False
        self.wall_follower_client.call_async(request)

    def can_leave_wall(self):
        # Determines if the robot can leave wall-following mode and return to goal navigation
        if datetime.now() - self.wall_following_start_time < self.min_wall_follow_duration:
            return False

        leave_distance = self.compute_distance(self.leave_point, self.target_position)
        current_distance = self.compute_distance(self.current_position, self.target_position)
        return current_distance + 0.2 < leave_distance

    def go_to_goal(self):
        # Executes navigation towards the target position
        desired_yaw = math.atan2(
            self.target_position.y - self.current_position.y,
            self.target_position.x - self.current_position.x
        )
        yaw_error = self.normalize_angle(desired_yaw - self.yaw)
        distance_error = self.compute_distance(self.current_position, self.target_position)

        if distance_error > 0.1:
            request = Gotopoint.Request()
            request.move_switch = True
            request.target_position = self.target_position
            self.go_to_point_client.call_async(request)
        else:
            self.get_logger().info("Goal reached! Deactivating Bug2 behavior.")
            self.is_active = False  # Deactivate Bug2 behavior

    @staticmethod
    def compute_distance(p1, p2):
        # Computes the Euclidean distance between two points
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    @staticmethod
    def normalize_angle(angle):
        # Normalizes an angle to the range [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    bug2_controller = Bug2Controller()
    rclpy.spin(bug2_controller)
    bug2_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
