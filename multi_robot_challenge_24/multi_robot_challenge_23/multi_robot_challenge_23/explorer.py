import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import time


class ExplorerController(Node):
    def __init__(self):
        super().__init__("explorer")
        

        self.scan = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(f'ExplorerController initialized for {self.get_namespace()}')



        #self.srv_explorer = self.create_service(SetBool, "/explorer", self.srv_callback)

        # Lidar readings
        self.lidar_left = 100
        self.lidar_leftfront = 100
        self.lidar_front = 100
        self.lidar_rightfront = 100
        self.lidar_right = 100

        self.wall_following_state = "search for wall"
         
        
        self.turn = 1.0
        



        self.speed = 0.60

        # distance from the wall.
        self.dist = 0.70
         
        # too close to the wall.
        self.dist_s = 0.40

        self.state = 0
        
        if self.get_namespace() == "/tb3_1":
            self.turn = self.turn
            self.spinLeft()

            #self.state = 1

        #self.active = False

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # def srv_callback(self, request, response):
    #     self.active = request.data
    #     response.success = True
    #     response.message = "Explorer"
    #     return response

    def spinLeft(self):
        vel_msg = Twist()
        vel_msg.angular.z = self.turn
        self.cmd_vel_pub.publish(vel_msg)
        time.sleep(2.6)
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)

    def timer_callback(self):
        
       # if not self.active:
       #     return

        vel_msg = Twist()
        d = self.dist
        #self.get_logger().info("Wall following state: " + self.wall_following_state)
         
        if self.lidar_leftfront > d and self.lidar_front > d and self.lidar_rightfront > d:
            self.wall_following_state = "search for wall"
            vel_msg.linear.x = self.speed
            vel_msg.angular.z = -0.01
            if self.state != 0:
                vel_msg.angular.z = -self.turn # turn right to find wall
                self.wall_following_state = "missing wall, turn right, search for wall"
             
        elif self.lidar_leftfront > d and self.lidar_front < d and self.lidar_rightfront > d:
            self.wall_following_state = "found wall, turn left"
            self.state = 1
            vel_msg.angular.z = self.turn
             
        elif (self.lidar_leftfront > d and self.lidar_front > d and self.lidar_rightfront < d):
            if (self.lidar_rightfront < self.dist_s):
                # Getting too close
                self.wall_following_state = "to close, turn left"
                vel_msg.linear.x = self.speed
                vel_msg.angular.z = self.turn      
            else:           
                # Go straight
                self.wall_following_state = "follow wall" 
                vel_msg.linear.x = self.speed   
                                     
        elif self.lidar_leftfront < d and self.lidar_front > d and self.lidar_rightfront > d:
            self.wall_following_state = "wall lf, missing f rf, turn left"
            vel_msg.angular.z = self.turn 
            self.state = 1
             
        elif self.lidar_leftfront > d and self.lidar_front < d and self.lidar_rightfront < d:
            self.wall_following_state = "wall f rf, missinf lf, turn left"
            vel_msg.angular.z = self.turn
            self.state = 1
             
        elif self.lidar_leftfront < d and self.lidar_front < d and self.lidar_rightfront < d:
            self.wall_following_state = "wall lf f rf, turn left"
            vel_msg.angular.z = self.turn
            self.state = 1
             
        elif self.lidar_leftfront < d and self.lidar_front < d and self.lidar_rightfront > d:
            self.wall_following_state = "wall lf f , missing rf, turn left"
            vel_msg.angular.z = self.turn
            self.state = 1
        self.cmd_vel_pub.publish(vel_msg)    


    def clbk_laser(self, msg):
        
        self.lidar_left = msg.ranges[90]
        self.lidar_leftfront = msg.ranges[45]
        self.lidar_front = msg.ranges[0]
        self.lidar_rightfront = msg.ranges[315]
        self.lidar_right = msg.ranges[270]

        # if self.get_namespace() == "/tb3_8":
        #     self.lidar_left = msg.ranges[270]
        #     self.lidar_leftfront = msg.ranges[315]
        #     self.lidar_front = msg.ranges[0]
        #     self.lidar_rightfront = msg.ranges[45]
        #     self.lidar_right = msg.ranges[90]

         
    

def main(args=None):
    rclpy.init(args=args)

    controller = ExplorerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()