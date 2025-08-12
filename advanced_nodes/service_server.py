import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.service = self.create_service(Trigger, 'amr_service', self.callback)

    def callback(self, request, response):
        response.success = True
        response.message = 'AMR Service Triggered Successfully'
        self.get_logger().info('Service Called')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
