import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import Int32
from example_interfaces.action import Fibonacci

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server')
        self.action_server = ActionServer(self, Fibonacci, 'amr_action',
                                          execute_callback=self.execute_callback)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
