import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from multi_robot_interfaces.action import ControlBehavior
from std_msgs.msg import Bool


class BehaviorControlActionServer(Node):
    def __init__(self):
        super().__init__('behavior_control_action_server')

        # Action Server
        self.action_server = ActionServer(
            self, ControlBehavior, '/control_behavior', self.execute_callback)

        # Publishers to control Bug2 and Explorer
        self.target_position_pub = self.create_publisher(Point, '/big_fire_position', 10)  # For Bug2 Controller
        self.explore_mode_pub = self.create_publisher(Bool, '/explore_mode', 10)  # For Explorer

        self.get_logger().info("Behavior Control Action Server initialized.")

    async def execute_callback(self, goal_handle):
        """
        Executes the requested behavior.
        """
        self.get_logger().info(f"Received goal: {goal_handle.request.behavior_name}")
        behavior_name = goal_handle.request.behavior_name
        target_position = goal_handle.request.target_position

        feedback = ControlBehavior.Feedback()
        result = ControlBehavior.Result()

        if behavior_name == "explore":
            # Switch to exploration mode
            self.get_logger().info("Switching to exploration mode.")
            explore_msg = Bool()
            explore_msg.data = True
            self.explore_mode_pub.publish(explore_msg)

            for i in range(10):  # Simulate exploration feedback
                if not goal_handle.is_active:
                    self.get_logger().info("Exploration canceled.")
                    result.status = -1
                    goal_handle.canceled()
                    return result

                feedback.feedback = f"Exploring... Step {i + 1}"
                self.get_logger().info(feedback.feedback)
                goal_handle.publish_feedback(feedback)
                await self.sleep(1)  # Simulate time delay

            # Exploration complete
            explore_msg.data = False
            self.explore_mode_pub.publish(explore_msg)
            self.get_logger().info("Exploration completed.")
            result.status = 1
            goal_handle.succeed()
            return result

        elif behavior_name == "go_to_point":
            if target_position is None:
                self.get_logger().error("Target position not provided for go_to_point.")
                result.status = -1
                goal_handle.abort()
                return result

            # Publish the target position for Bug2 Controller
            self.get_logger().info(f"Navigating to target position: {target_position}")
            self.target_position_pub.publish(target_position)

            feedback.feedback = f"Navigating to {target_position.x}, {target_position.y}"
            goal_handle.publish_feedback(feedback)

            # Simulate navigation duration (replace with actual feedback from Bug2 if needed)
            await self.sleep(10)

            self.get_logger().info("Reached target position.")
            result.status = 1
            goal_handle.succeed()
            return result

        else:
            self.get_logger().error(f"Unknown behavior: {behavior_name}")
            result.status = -1
            goal_handle.abort()
            return result

    async def sleep(self, duration):
        """
        Helper function to simulate asynchronous sleep.
        """
        await rclpy.task.Future().delayed(duration)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorControlActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
