import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from bug2_interfaces.srv import Gotopoint
from std_srvs.srv import SetBool
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from datetime import datetime, timedelta  # For timing

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')
        
        # Initialize service clients for wall follower and go-to-point
        self.wall_follower_client = self.create_client(SetBool, '/wallfollower')
        self.wall_follower_client.wait_for_service()
        
        self.go_to_point_client = self.create_client(Gotopoint, '/gotopoint')
        self.go_to_point_client.wait_for_service()

        # Set up initial variables
        self.goal_position = Point()  # Set your goal position
        self.goal_position.x = -7.0
        self.goal_position.y = 2.0
        self.goal_position.z = 0.0

        self.current_position = Point()  # Update this from odometry
        self.is_wall_following = False
        self.leave_point = None
        self.distance = float('inf')
        self.obstacle_detected = False

        # Time buffer for wall following
        self.wall_following_start_time = None
        self.min_wall_follow_duration = timedelta(seconds=10)  # Stay in wall-following for 10 seconds

        # Timer for periodic checks
        self.timer_preiod = 1.0
        self.timer = self.create_timer(self.timer_preiod, self.timer_callback)

        self.goto_sub = self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)

        self.closer_buffer = 0.2

    def clbk_laser(self, msg):
        self.front = min(msg.ranges[0:20] + msg.ranges[340:360])
        self.right = min(msg.ranges[70:180])

        self.obstacle_detected = self.front < 1.0

    def clbk_odom(self, msg):
        position = msg.pose.pose.position
        self.current_position = Point()
        self.current_position.x = position.x
        self.current_position.y = position.y
        self.current_position.z = position.z

        if self.current_position:
            self.distance = self.is_closer_to_goal(self.current_position, self.goal_position)

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def timer_callback(self):
        if not self.obstacle_detected:
            # No obstacle detected, continue go-to-point
            self.start_go_to_point(True)

        elif self.obstacle_detected:
            if not self.is_wall_following:
                # Start wall-following mode
                self.start_go_to_point(False)
                self.start_wall_following(True)
                self.is_wall_following = True
                self.leave_point = self.current_position
                self.wall_following_start_time = datetime.now()  # Store the start time
            else:
                # If wall-following, check if it's time to switch back to go-to-point
                current_time = datetime.now()
                if current_time - self.wall_following_start_time >= self.min_wall_follow_duration:
                    hit_distance = self.is_closer_to_goal(self.current_position, self.goal_position)
                    leave_distance = self.is_closer_to_goal(self.leave_point, self.goal_position)

                    if hit_distance + self.closer_buffer < leave_distance:
                        self.get_logger().info('Obstacle cleared, switching to go-to-point.')
                        self.is_wall_following = False
                        self.start_wall_following(False)
                        self.start_go_to_point(True)
                    else:
                        self.get_logger().info('Continuing wall-following until closer to the goal.')

    def start_wall_following(self, running):
        self.get_logger().info(f'{"Starting" if running else "Stopping"} wall-following.')
        request = SetBool.Request()
        request.data = running
        self.wall_follower_client.call_async(request)

    def start_go_to_point(self, running):
        self.get_logger().info(f'{"Starting" if running else "Stopping"} GoToPoint mode.')
        request = Gotopoint.Request()
        request.move_switch = running
        request.target_position = self.goal_position
        self.go_to_point_client.call_async(request)

    def is_closer_to_goal(self, p1, p2):
        # Compare distances from hit point and leave point to the goal
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def distance_to_line(self, p0):
        # p0 is the current position (the point from which we calculate the distance)
        # p1 and p2 define the line (from start to goal position)
        p1 = self.start_position
        p2 = self.goal_position

        # Calculating the numerator and denominator of the distance formula
        up_eq = abs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + p2.x * p1.y - p2.y * p1.x)
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))

        # Calculate and return the distance
        distance = up_eq / lo_eq
        return distance


def main(args=None):
    rclpy.init(args=args)
    bug2_controller = Bug2Controller()
    rclpy.spin(bug2_controller)
    bug2_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
