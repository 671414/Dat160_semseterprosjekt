
from multi_robot_challenge_23.explorer import ExplorerController
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.logging import get_logger


class Controller(Node):
    def __init__(self):
        super().__init__('master_controller')

        self.robot_namespaces = ['tb3_0', 'tb3_1']

        self.robots = {}
        for namespace in self.robot_namespaces:
            self._logger.info(f'Initializing classes for {namespace}...')
            self.robots[namespace] = {
                'Explorer': ExplorerController(namespace),        
            }

            self._logger.info('All classes initialized successfully for both robots.')





def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()