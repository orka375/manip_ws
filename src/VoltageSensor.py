#!/usr/bin/env python3

import time
import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# I2C and ADS1115 Setup
I2C_BUS = 0
ADS1115_ADDR = 0x49
REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# ADS1115 config bits
GAIN = 0x0200     # ±4.096V range
MODE = 0x0100     # Single-shot mode
SPS = 0x0080      # 128 samples per second
COMP = 0x0003     # Disable comparator
OS = 0x8000       # Start conversion
MUX = 0x5000      # AIN1 (single-ended) = MUX[2:0] = 101

# Final config word for AIN1
CONFIG = OS | MUX | GAIN | MODE | SPS | COMP

# Calibration constants based on your measurements
ADC_AT_0V = 2.5
ADC_AT_24V = 2.879
VOLTAGE_SPAN = 24.0
GAIN_CALC = VOLTAGE_SPAN / (ADC_AT_24V - ADC_AT_0V)

class VoltagePublisher(Node):
    def __init__(self):
        super().__init__('vmeter_node')
        self.publisher_ = self.create_publisher(Float32, '/vmeter_voltage', 10)
        self.timer = self.create_timer(1.0, self.read_and_publish_voltage)

        # Open I2C connection
        self.i2c = lgpio.i2c_open(I2C_BUS, ADS1115_ADDR)

    def read_and_publish_voltage(self):
        try:
            # Write config to start conversion
            config_bytes = [(CONFIG >> 8) & 0xFF, CONFIG & 0xFF]
            lgpio.i2c_write_device(self.i2c, bytes([REG_CONFIG] + config_bytes))
            time.sleep(0.01)  # Wait for conversion

            # Read conversion result
            lgpio.i2c_write_device(self.i2c, [REG_CONVERSION])
            _, data = lgpio.i2c_read_device(self.i2c, 2)
            raw = (data[0] << 8) | data[1]
            if raw > 0x7FFF:
                raw -= 1 << 16

            voltage = raw * 4.096 / 32768  # Convert to volts
            real_voltage = (voltage - ADC_AT_0V) * GAIN_CALC

            msg = Float32()
            msg.data = real_voltage
            self.publisher_.publish(msg)
            self.get_logger().info(f'AIN1: {voltage:.4f} V → Measured: {real_voltage:.2f} V')

        except Exception as e:
            self.get_logger().error(f"Error reading sensor: {e}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoltagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
