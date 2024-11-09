import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64
from scoring_interfaces.srv import SetMarkerPosition


class MarkerReporter(Node):
    def __init__(self):
        super().__init__('marker_reporter')

        # Retrieve namespace parameter
        self.declare_parameter('namespace', '')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        if not self.namespace:
            self.get_logger().error("Namespace parameter is not set. Exiting...")
            raise ValueError("Namespace parameter is required")

        # Marker data for this namespace
        self.marker_data = {"pose": None, "id": None}
        self.reported_markers = set()

        # Service client
        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_marker_position service...')




        # Subscriptions for marker data in this namespace
        self.create_subscription(Pose, f'/{self.namespace}/marker_map_pose', self.marker_pose_callback, 10)
        self.create_subscription(Int64, f'/{self.namespace}/marker_id', self.marker_id_callback, 10)




        # Timer for processing markers
        self.timer = self.create_timer(1.5, self.timer_callback)

    def marker_pose_callback(self, msg):
        self.marker_data["pose"] = msg
        #self.get_logger().info(f"[{self.namespace}] Marker position updated: {msg.position}")





    def marker_id_callback(self, msg):
        self.marker_data["id"] = int(msg.data)
        #self.get_logger().info(f"[{self.namespace}] Marker ID received: {msg.data}")






    def timer_callback(self):
        if self.marker_data["id"] is None or self.marker_data["pose"] is None:
            self.get_logger().info(f"[{self.namespace}] No valid marker data to report.")
            return

        if self.marker_data["id"] in self.reported_markers:
            self.get_logger().info(f"[{self.namespace}] Marker ID {self.marker_data['id']} already reported. Skipping...")
            return

        self.report_marker()




    def report_marker(self):
        req = SetMarkerPosition.Request()
        req.marker_id = self.marker_data["id"]
        req.marker_position = self.marker_data["pose"].position


        self.get_logger().info(f"[{self.namespace}] Reporting marker ID {req.marker_id} at position {req.marker_position}")
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_service_response)




    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f"[{self.namespace}] Marker successfully reported.")
                self.reported_markers.add(self.marker_data["id"])
                self.marker_data = {"pose": None, "id": None}  # Reset after reporting
            else:
                self.get_logger().info(f"[{self.namespace}] Marker reporting failed.")
        except Exception as e:
            self.get_logger().error(f"[{self.namespace}] Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
