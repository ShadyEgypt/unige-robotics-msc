#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.handle_add_two_ints
        )
        self.get_logger().info('AddTwoIntsServer node started')

    def handle_add_two_ints(self, request, response):
        self.get_logger().info(f'Received request: {request.a} + {request.b}')
        response.sum = request.a + request.b
        self.get_logger().info(f'Responding with: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
