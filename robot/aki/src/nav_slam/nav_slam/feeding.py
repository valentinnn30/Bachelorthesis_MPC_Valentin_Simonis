import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import asyncio
import time

from slam_interface.action import ActivateSlam


class FeedingNode(Node):
    def __init__(self):
        super().__init__('feeding_server')

        self.command_pub = self.create_publisher(Int32, '/feeding_command', 10)
        self.feedback_sub = self.create_subscription(Int32, '/feeding_result', self.feedback_callback, 10)
        self._action_server = ActionServer(self, ActivateSlam,'feeding', self.execute_callback)


        self.current_goal_handle = None
        self.expected_value = None
        self._result_future = None
        self.feeding_bool = 0

    def feedback_callback(self, msg):
        self.feeding_bool = msg.data
        # self.get_logger().info(f"Received feedback: {msg.data}")



    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received request to {'feed' if goal_handle.request.activate else 'no feeding'}")

        self.current_goal_handle = goal_handle
        self.expected_value = 10 if goal_handle.request.activate else 0

        msg = Int32()
        msg.data = self.expected_value
        self.command_pub.publish(msg)
        self.expected_value = None

        feedback_msg = ActivateSlam.Feedback()
        feedback_msg.status = f"Sent activation command for feeding: {msg.data}, waiting for feedback"
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
        msg.data = 0
        self.command_pub.publish(msg)
        time.sleep(1.0)

        while self.feeding_bool != 1:
            time.sleep(0.1)
        self.get_logger().info("Feeding process completed successfully")

        self.current_goal_handle.succeed()
        result = ActivateSlam.Result()
        result.success = True
     
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FeedingNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




