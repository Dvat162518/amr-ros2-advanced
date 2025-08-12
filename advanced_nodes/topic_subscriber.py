import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

class TopicSubscriber(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.subscription = self.create_subscription(String, 'amr_topic', self.callback, qos)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TopicSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
