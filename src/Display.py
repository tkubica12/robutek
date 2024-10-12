from machine import I2C, Pin
import ssd1306

class Display:
    def __init__(self, i2c: I2C):
        i2c = i2c
        oled_width = 128
        oled_height = 64
        self.oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
        self.messages = {0: "", 1: "", 2: "", 3: ""}

    def show_message(self, message: str, line: int):
        if self.messages[line] != message:
            self.messages[line] = message
            self.oled.fill(0)
            for line, message in self.messages.items():
                self.oled.text(message, 0, line * 12)
            self.oled.show()