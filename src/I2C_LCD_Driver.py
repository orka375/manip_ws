import time
import gc
import lgpio

# PCF8574 pin definitions
MASK_RS = 0x01       # P0
MASK_RW = 0x02       # P1
MASK_E  = 0x04       # P2

SHIFT_BACKLIGHT = 3  # P3
SHIFT_DATA      = 4  # P4-P7

class LcdApi:
    # HD44780 LCD controller command set
    LCD_CLR             = 0x01
    LCD_HOME            = 0x02
    LCD_ENTRY_MODE      = 0x04
    LCD_ENTRY_INC       = 0x02
    LCD_ENTRY_SHIFT     = 0x01
    LCD_ON_CTRL         = 0x08
    LCD_ON_DISPLAY      = 0x04
    LCD_ON_CURSOR       = 0x02
    LCD_ON_BLINK        = 0x01
    LCD_MOVE            = 0x10
    LCD_MOVE_DISP       = 0x08
    LCD_MOVE_RIGHT      = 0x04
    LCD_FUNCTION        = 0x20
    LCD_FUNCTION_8BIT   = 0x10
    LCD_FUNCTION_2LINES = 0x08
    LCD_FUNCTION_10DOTS = 0x04
    LCD_FUNCTION_RESET  = 0x30
    LCD_CGRAM           = 0x40
    LCD_DDRAM           = 0x80
    LCD_RS_CMD          = 0
    LCD_RS_DATA         = 1
    LCD_RW_WRITE        = 0
    LCD_RW_READ         = 1

    def __init__(self, num_lines, num_columns):
        self.num_lines = min(num_lines, 4)
        self.num_columns = min(num_columns, 40)
        self.cursor_x = 0
        self.cursor_y = 0
        self.implied_newline = False
        self.backlight = True
        self.display_off()
        self.backlight_on()
        self.clear()
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)
        self.hide_cursor()
        self.display_on()

    def clear(self):
        self.hal_write_command(self.LCD_CLR)
        self.hal_write_command(self.LCD_HOME)
        self.cursor_x = 0
        self.cursor_y = 0

    def show_cursor(self):
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY | self.LCD_ON_CURSOR)

    def hide_cursor(self):
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def blink_cursor_on(self):
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY | self.LCD_ON_CURSOR | self.LCD_ON_BLINK)

    def blink_cursor_off(self):
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY | self.LCD_ON_CURSOR)

    def display_on(self):
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def display_off(self):
        self.hal_write_command(self.LCD_ON_CTRL)

    def backlight_on(self):
        self.backlight = True
        self.hal_backlight_on()

    def backlight_off(self):
        self.backlight = False
        self.hal_backlight_off()

    def move_to(self, cursor_x, cursor_y):
        self.cursor_x = cursor_x
        self.cursor_y = cursor_y
        addr = cursor_x & 0x3f
        if cursor_y & 1:
            addr += 0x40
        if cursor_y & 2:
            addr += self.num_columns
        self.hal_write_command(self.LCD_DDRAM | addr)

    def putchar(self, char):
        if char == '\n':
            if not self.implied_newline:
                self.cursor_x = self.num_columns
        else:
            self.hal_write_data(ord(char))
            self.cursor_x += 1
        if self.cursor_x >= self.num_columns:
            self.cursor_x = 0
            self.cursor_y += 1
            self.implied_newline = (char != '\n')
        if self.cursor_y >= self.num_lines:
            self.cursor_y = 0
        self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        for char in string:
            self.putchar(char)

    def custom_char(self, location, charmap):
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        self.hal_sleep_us(40)
        for i in range(8):
            self.hal_write_data(charmap[i])
            self.hal_sleep_us(40)
        self.move_to(self.cursor_x, self.cursor_y)

    def hal_backlight_on(self):
        pass

    def hal_backlight_off(self):
        pass

    def hal_write_command(self, cmd):
        raise NotImplementedError

    def hal_write_data(self, data):
        raise NotImplementedError

    def hal_sleep_us(self, usecs):
        time.sleep(usecs / 1_000_000.0)

class I2cLcd(LcdApi):
    def __init__(self, num_lines, num_columns):
        addr = 0x27
        self.h = lgpio.i2c_open(0, addr)
        lgpio.i2c_write_byte(self.h, 0)
        time.sleep(0.020)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        time.sleep(0.005)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        time.sleep(0.001)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        time.sleep(0.001)
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        time.sleep(0.001)
        super().__init__(num_lines, num_columns)
        cmd = self.LCD_FUNCTION
        if num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)
        gc.collect()

    def hal_write_init_nibble(self, nibble):
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        lgpio.i2c_write_byte(self.h, byte | MASK_E)
        lgpio.i2c_write_byte(self.h, byte)
        gc.collect()

    def hal_backlight_on(self):
        lgpio.i2c_write_byte(self.h, 1 << SHIFT_BACKLIGHT)
        gc.collect()

    def hal_backlight_off(self):
        lgpio.i2c_write_byte(self.h, 0)
        gc.collect()

    def hal_write_command(self, cmd):
        byte = ((self.backlight << SHIFT_BACKLIGHT) | (((cmd >> 4) & 0x0f) << SHIFT_DATA))
        lgpio.i2c_write_byte(self.h, byte | MASK_E)
        lgpio.i2c_write_byte(self.h, byte)
        byte = ((self.backlight << SHIFT_BACKLIGHT) | ((cmd & 0x0f) << SHIFT_DATA))
        lgpio.i2c_write_byte(self.h, byte | MASK_E)
        lgpio.i2c_write_byte(self.h, byte)
        if cmd <= 3:
            time.sleep(0.005)
        gc.collect()

    def hal_write_data(self, data):
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | (((data >> 4) & 0x0f) << SHIFT_DATA))
        lgpio.i2c_write_byte(self.h, byte | MASK_E)
        lgpio.i2c_write_byte(self.h, byte)
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | ((data & 0x0f) << SHIFT_DATA))
        lgpio.i2c_write_byte(self.h, byte | MASK_E)
        lgpio.i2c_write_byte(self.h, byte)
        gc.collect()
