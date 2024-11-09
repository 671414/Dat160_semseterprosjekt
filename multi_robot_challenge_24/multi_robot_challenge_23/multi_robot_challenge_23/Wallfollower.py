import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool  # Import SetBool service type

class WallFollowerClass(Node):
    def __init__(self):
        super().__init__('WallFollowerController')

        self.scan_sub_tb3_0 = self.create_subscription(LaserScan, '/tb3_0/scan', self.clbk_laser_tb3_0, 10)
        self.scan_sub_tb3_1 = self.create_subscription(LaserScan, '/tb3_1/scan', self.clbk_laser_tb3_1, 10)
        
        self.vel_pub_tb3_0 = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)
        self.vel_pub_tb3_1 = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)

        # Create Service Server Definition
        self.wall_follower_service = self.create_service(SetBool, '/wallfollower', self.wall_follower_callback)

        self.active = True

        self.regions = {
            'right': 0.0,
            'fright': 0.0,
            'front': 0.0,
            'fleft': 0.0,
            'left': 0.0,
        }
        self.state = 0
        self.state_dict = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    #TODO: Create Callback Function for the Service Server that sets the self.active variable True or False depending on the request message
    def wall_follower_callback(self, request, response):
        # Set the self.active variable based on the request
        self.active = request.data
        response.success = True
        response.message = f"wallfollowing: {self.active}"
        return response


    #ADDED during project work Andrè
    def clbk_laser_tb3_0(self, msg):
        self.regions = {
            'right':  min(min(msg.ranges[180:299]), 1.0),
            'fright': min(min(msg.ranges[320:339]), 1.0),
            'front':  min(min(min(msg.ranges[0:9]), min(msg.ranges[350:359])), 1.0),
            'fleft':  min(min(msg.ranges[20:39]), 1.0),
            'left':   min(min(msg.ranges[60:179]), 1.0),
        }
        print(self.regions)
        self.take_action()
    
    #ADDED during project work Andrè
    def clbk_laser_tb3_1(self, msg):
        self.regions = {
            'right':  min(min(msg.ranges[180:299]), 1.0),
            'fright': min(min(msg.ranges[320:339]), 1.0),
            'front':  min(min(min(msg.ranges[0:9]), min(msg.ranges[350:359])), 1.0),
            'fleft':  min(min(msg.ranges[20:39]), 1.0),
            'left':   min(min(msg.ranges[60:179]), 1.0),
        }
        print(self.regions)
        self.take_action()
        

    def change_state(self, state):
        if state != self.state:
            self.get_logger().info('Wall follower - [' + str(state) + '] - ' + str(self.state_dict[state]))
            self.state = state

    def take_action(self):
        regions = self.regions

        d = 0.9

        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            self.change_state(0)  # Case 1: nothing
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            self.change_state(1)  # Case 2: front
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            self.change_state(2)  # Case 3: fright
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            self.change_state(0)  # Case 4: fleft
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            self.change_state(1)  # Case 5: front and fright
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            self.change_state(1)  # Case 6: front and fleft
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            self.change_state(1)  # Case 7: front and fleft and fright
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            self.change_state(0)  # Case 8: fleft and fright
        else:
            self.get_logger().info('unknown case')

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = -0.5
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.5
        return msg

    #Changed during project work Andrè
    def timer_callback(self):
        if not self.active:
            return
        msg = Twist()
        if self.state == 0:
            msg = self.find_wall()
        elif self.state == 1:
            msg = self.turn_left()
        elif self.state == 2:
            msg = self.follow_the_wall()
        else:
            self.get_logger().error('Unknown state!')

        self.vel_pub_tb3_0.publish(msg)
        self.vel_pub_tb3_1.publish(msg)

    

def main(args=None):
    rclpy.init(args=args)

    controller = WallFollowerClass()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()