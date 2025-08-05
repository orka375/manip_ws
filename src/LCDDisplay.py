import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import netifaces
from std_srvs.srv import Trigger



from src import I2C_LCD_Driver

DISPLAY_TIME = 4  # seconds to display each message
INACTIVITY_TIMEOUT = 10  # seconds to turn off display if no new messages

class LCDDisplayNode(Node):
    def __init__(self):
        super().__init__('lcd_display_node')

        self.lcd = I2C_LCD_Driver.I2cLcd(2, 16)
        self.lcd.clear()

        self.line1 = ""
        self.line2 = ""
        self.last_line1 = ""
        self.last_line2 = ""

        self.lock = threading.Lock()
        self.last_message_time = time.time()
        self.display_on = True
        self.publisher_top = self.create_publisher(String, 'lcd_line1', 10)
        self.publisher_bottom = self.create_publisher(String, 'lcd_line2', 10)


        self.srv_predef1 = self.create_service(Trigger, 'lcd_show_ip', self.get_and_show_ip)

        self.sub_line1 = self.create_subscription(String, 'lcd_line1', self.line1_callback, 10)
        self.sub_line2 = self.create_subscription(String, 'lcd_line2', self.line2_callback, 10)

        self.display_thread = threading.Thread(target=self.display_worker, daemon=True)
        self.display_thread.start()

    def line1_callback(self, msg):
        with self.lock:
            self.line1 = msg.data[:16]  # Truncate if too long
            self.last_message_time = time.time()
            if not self.display_on:
                self.lcd.display_on()
                self.lcd.backlight_on()
                self.display_on = True

    def line2_callback(self, msg):
        with self.lock:
            self.line2 = msg.data[:16]
            self.last_message_time = time.time()
            if not self.display_on:
                self.lcd.display_on()
                self.lcd.backlight_on()
                self.display_on = True

    def display_worker(self):
        while True:
            try:
                if self.display_on:
                    with self.lock:
                        l1 = self.line1
                        l2 = self.line2

                    if l1 != self.last_line1 or l2 != self.last_line2:
                        self.lcd.clear()
                        self.lcd.move_to(0, 0)
                        self.lcd.putstr(l1.ljust(16))
                        self.lcd.move_to(0, 1)
                        self.lcd.putstr(l2.ljust(16))
                        self.last_line1 = l1
                        self.last_line2 = l2

                    time.sleep(DISPLAY_TIME)

                # Check inactivity timeout
                if time.time() - self.last_message_time > INACTIVITY_TIMEOUT:
                    if self.display_on:
                        self.lcd.clear()
                        self.lcd.display_off()
                        self.lcd.backlight_off()
                        self.display_on = False

            except Exception as e:
                print(f"[LCD ERROR] {e}")
                time.sleep(1)



    #------------Defined Scripts to display  ------------
       
    def get_and_show_ip(self,request,response):
        try:
 

            ainterf = ["wlan0"]

            for i in ainterf:
                addresses = netifaces.ifaddresses(i)
                ip_info = addresses[netifaces.AF_INET][0]
                _ip =  ip_info['addr']
                self.get_logger().info(f"{_ip}")
                #Pub
                msg = String()
                msg.data = i
                self.publisher_top.publish(msg)
                time.sleep(0.05)
                msg = String()
                msg.data = _ip
                self.publisher_bottom.publish(msg)
                time.sleep(0.05)
            response.success = True
            self.get_logger().info(f"ShowIPService called. Responding with: {response.message}")
            return response
        

        except (KeyError, IndexError):
            response.success = False
            response.message = "Could not retrieve IP address"
            return response  # ✅ Always return a valid response

            





def main(args=None):
    rclpy.init(args=args)
    node = LCDDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.lcd.clear()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
