import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(Trigger, 'amr_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.request = Trigger.Request()
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
