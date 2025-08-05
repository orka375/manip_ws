import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your message type

class MessageLogger(Node):
    def __init__(self):
        super().__init__('message_logger')
        self.subscription = self.create_subscription(
            String,  # Change to your message type
            '/Log',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'---: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MessageLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
