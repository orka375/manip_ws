import time
import lgpio


# Constants
I2C_BUS = 0
ADS1115_ADDR = 0x49

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# ADS1115 settings
GAIN = 0x0200  # ±4.096V
MODE = 0x0100  # Single-shot
SPS = 0x0080   # 128 samples per second
COMP = 0x0003  # Disable comparator
OS = 0x8000    # Start conversion

# Create GPIO chip handle and open I2C

i2c = lgpio.i2c_open(I2C_BUS, ADS1115_ADDR)

def get_config(channel):
    mux_map = {
        0: 0x4000,
        1: 0x5000,
        2: 0x6000,
        3: 0x7000
    }
    return OS | mux_map[channel] | GAIN | MODE | SPS | COMP

try:
    while True:
        for ch in range(4):
            # Configure and start conversion
            config = get_config(ch)
            config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
            lgpio.i2c_write_device(i2c, [REG_CONFIG] + config_bytes)

            # Wait for conversion to complete (8ms is safe for 128SPS)
            time.sleep(0.01)

            # Read conversion result
            lgpio.i2c_write_device(i2c, [REG_CONVERSION])
            _, data = lgpio.i2c_read_device(i2c, 2)
            raw = (data[0] << 8) | data[1]
            if raw > 0x7FFF:
                raw -= 1 << 16

            voltage = raw * 4.096 / 32768  # LSB = 125uV
            # Calibration endpoints
            adc_at_0v = 2.5
            adc_at_24v = 2.879
            volt_span = 24

            gain = volt_span / (adc_at_24v - adc_at_0v)  # ≈ 63.33
            real_voltage = (voltage - adc_at_0v) * gain

            print(f"Channel {ch}: voltage = {voltage:6f}, realVoltage = {real_voltage:.6f} V")

        print("-" * 40)
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")

