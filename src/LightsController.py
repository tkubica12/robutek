from neopixel import NeoPixel
from Components import Light

class LightsController:
    COLOR_WHITE = (255, 255, 255)
    COLOR_DIM_WHITE = (32, 32, 32)
    COLOR_BLACK = (0, 0, 0)
    COLOR_RED = (255, 0, 0)
    COLOR_DIM_RED = (32, 0, 0)
    COLOR_GREEN = (0, 255, 0)
    COLOR_YELLOW = (255, 255, 0)

    def __init__(self, np: NeoPixel):
        self.left_front_1 = Light(np=np, index=0)
        self.left_front_2 = Light(np=np, index=1)
        self.right_front_1 = Light(np=np, index=2)
        self.right_front_2 = Light(np=np, index=3)
        self.left_back_1 = Light(np=np, index=4)
        self.left_back_2 = Light(np=np, index=5)
        self.right_back_1 = Light(np=np, index=6)
        self.right_back_2 = Light(np=np, index=7)
        self.np = np

    def break_on(self):
        self.left_back_1.turn_on(color=self.COLOR_RED, blinking=False)
        self.left_back_2.turn_on(color=self.COLOR_RED, blinking=False)
        self.right_back_1.turn_on(color=self.COLOR_RED, blinking=False)
        self.right_back_2.turn_on(color=self.COLOR_RED, blinking=False)

        # Process
        self.process()

    def break_off(self):
        self.left_back_1.turn_off()
        self.left_back_2.turn_off()
        self.right_back_1.turn_off()
        self.right_back_2.turn_off()

        # Turn main lights on
        self.main_on()

    def main_on(self):
        # Set all front lights to white
        self.left_front_1.turn_on(color=self.COLOR_WHITE, blinking=False)
        self.left_front_2.turn_on(color=self.COLOR_WHITE, blinking=False)
        self.right_front_1.turn_on(color=self.COLOR_WHITE, blinking=False)
        self.right_front_2.turn_on(color=self.COLOR_WHITE, blinking=False)

        # Set all back to dim red
        self.left_back_1.turn_on(color=self.COLOR_DIM_RED, blinking=False)
        self.left_back_2.turn_on(color=self.COLOR_DIM_RED, blinking=False)
        self.right_back_1.turn_on(color=self.COLOR_DIM_RED, blinking=False)
        self.right_back_2.turn_on(color=self.COLOR_DIM_RED, blinking=False)

        # Process
        self.process()

    def main_off(self):
        # Turn all lights off
        self.left_front_1.turn_off()
        self.left_front_2.turn_off()
        self.right_front_1.turn_off()
        self.right_front_2.turn_off()
        self.left_back_1.turn_off()
        self.left_back_2.turn_off()
        self.right_back_1.turn_off()
        self.right_back_2.turn_off()

        # Process
        self.process()

    def left_blinker_on(self):
        self.left_front_2.turn_on(color=self.COLOR_YELLOW, blinking=True)
        self.left_back_1.turn_on(color=self.COLOR_YELLOW, blinking=True)

        # Process
        self.process()

    def left_blinker_off(self):
        self.left_front_2.turn_off()
        self.left_back_1.turn_off()
        self.main_on()

    def right_blinker_on(self):
        self.right_front_1.turn_on(color=self.COLOR_YELLOW, blinking=True)
        self.right_back_2.turn_on(color=self.COLOR_YELLOW, blinking=True)

        # Process
        self.process()

    def right_blinker_off(self):
        self.right_front_1.turn_off()
        self.right_back_2.turn_off()
        self.main_on()

    def reverse_on(self):
        self.left_back_1.turn_on(color=self.COLOR_WHITE, blinking=False)

        # Process
        self.process()

    def reverse_off(self):
        self.left_back_1.turn_off()
        self.main_on()

    def hazard_on(self):
        self.left_blinker_on()
        self.right_blinker_on()

    def hazard_off(self):
        self.left_blinker_off()
        self.right_blinker_off()

    def test_on(self):
        # Turn all lights on using white color and blinking
        self.left_front_1.turn_on(color=self.COLOR_WHITE, blinking=True)
        self.left_front_2.turn_on(color=self.COLOR_WHITE, blinking=True)
        self.right_front_1.turn_on(color=self.COLOR_WHITE, blinking=True)
        self.right_front_2.turn_on(color=self.COLOR_WHITE, blinking=True)
        self.left_back_1.turn_on(color=self.COLOR_WHITE, blinking=True)
        self.left_back_2.turn_on(color=self.COLOR_WHITE, blinking=True)
        self.right_back_1.turn_on(color=self.COLOR_WHITE, blinking=True)
        self.right_back_2.turn_on(color=self.COLOR_WHITE, blinking=True)

        # Process
        self.process()

    def test_off(self):
        # Turn all lights off
        self.left_front_1.turn_off()
        self.left_front_2.turn_off()
        self.right_front_1.turn_off()
        self.right_front_2.turn_off()
        self.left_back_1.turn_off()
        self.left_back_2.turn_off()
        self.right_back_1.turn_off()
        self.right_back_2.turn_off()

        # Process
        self.process()

    def process(self, timer=None):
        # Process all lights
        self.left_front_1.process()
        self.left_front_2.process()
        self.right_front_1.process()
        self.right_front_2.process()
        self.left_back_1.process()
        self.left_back_2.process()
        self.right_back_1.process()
        self.right_back_2.process()

        # Write to NeoPixel
        self.np.write()
