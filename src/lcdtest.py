from I2C_LCD_Driver import I2cLcd
import time
import netifaces

lcd = I2cLcd(2, 16)  # Adjust bus and address
lcd.backlight_on()
lcd.clear()
lcd.move_to(0, 0)
lcd.putstr("Welcome to")
lcd.move_to(0, 1)
lcd.putstr("Youbot")
time.sleep(5)
lcd.clear()


ainterf = ["wlan0"]

for i in ainterf:
    addresses = netifaces.ifaddresses(i)
    ip_info = addresses[netifaces.AF_INET][0]
    _ip =  ip_info['addr']

    lcd.move_to(0, 0)
    lcd.putstr("WLAN0")
    lcd.move_to(0, 1)
    lcd.putstr(_ip)
    time.sleep(5)
    lcd.clear() 
lcd.backlight_off()

            