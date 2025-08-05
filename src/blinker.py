# import rclpy
# from rclpy.node import Node
# import lgpio
# import time

# class LedBlinker(Node):
#     def __init__(self):
#         super().__init__('led_blinker')
#         self.LED = 23
#         self.h = lgpio.gpiochip_open(0)
#         lgpio.gpio_claim_output(self.h, self.LED)
#         self.get_logger().info('LED Blinker Node Started')

#         # Timer every 1 second
#         self.led_on = False
#         self.timer = self.create_timer(1.0, self.blink_led)

#     def blink_led(self):
#         self.led_on = not self.led_on
#         lgpio.gpio_write(self.h, self.LED, int(self.led_on))
#         self.get_logger().info(f"LED {'ON' if self.led_on else 'OFF'}")

#     def destroy_node(self):
#         lgpio.gpio_write(self.h, self.LED, 0)
#         lgpio.gpiochip_close(self.h)
#         self.get_logger().info('Cleaned up GPIO')
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = LedBlinker()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

