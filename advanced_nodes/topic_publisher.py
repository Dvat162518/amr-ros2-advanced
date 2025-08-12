import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.publisher = self.create_publisher(String, 'amr_topic', qos)
        self.timer = self.create_timer(1.0, self.publish_callback)
        self.count = 0

    def publish_callback(self):
        msg = String()
        msg.data = f'AMR Status Update {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TopicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
