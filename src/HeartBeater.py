import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class HeartBeater(Node):
    def __init__(self):
        super().__init__('bool_toggle_node')

        # Publisher for /toggle_bool
        self.publisher_ = self.create_publisher(Bool, 'my_heart_beat', 10)

        # Timer: 1 second interval
        self.timer = self.create_timer(8.0, self.timer_callback)

        # Toggle state
        self.current_state = False

        self.get_logger().info("🔁 Heart Beater node started (publishing every second)")

    def timer_callback(self):
        self.current_state = not self.current_state
        msg = Bool()
        msg.data = self.current_state
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = HeartBeater()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down HeartBeater node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
