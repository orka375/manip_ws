import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import lgpio


class RelayControlNode(Node):
    def __init__(self):
        super().__init__('relay_control_node')

        # self.gpio_chip = lgpio.gpiochip_open(0)

        # Named pin map: service name -> GPIO number
        self.pin_map = {
            'PowerBase': 5,
            'PowerArm': 6,
            'Res1': 26,
            'Res2': 25
        }

        # Setup all pins as outputs
        # for pin in self.pin_map.values():
        #     lgpio.gpio_claim_output(self.gpio_chip, pin, 0)

        # Create a service for each pin
        for name in self.pin_map:
            self.create_service(
                SetBool,
                f'/set_{name}',
                self.make_service_callback(name)
            )
            self.get_logger().info(f'Service ready: /set_{name}')

    def make_service_callback(self, name):
        def callback(request, response):
            # gpio_pin = self.pin_map[name]
            # value = 1 if request.data else 0
            # lgpio.gpio_write(self.gpio_chip, gpio_pin, value)
            response.success = True
            # response.message = f'{name} (GPIO {gpio_pin}) set to {value}'
            response.message = "Success"
            # self.get_logger().info(response.message)
            return response

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = RelayControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # lgpio.gpiochip_close(node.gpio_chip)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
