from MotorsController import MotorsController
from LightsController import LightsController
from MovementController import MovementController
from MotorsController import Direction
from Display import Display
from neopixel import NeoPixel
from machine import I2C, Pin, PWM
from RobotStateMachineStates import *
from LineTracking import LineTracking
from Path import Path, PathAction

class Robot:
    def __init__(self, np: NeoPixel, i2c: I2C, left_motor_forward_pwm_pin: PWM, left_motor_backward_pwm_pin: PWM, right_motor_forward_pwm_pin: PWM, right_motor_backward_pwm_pin: PWM, path_actions: list[PathAction] = []):
        self.motors_controller = MotorsController(left_motor_forward_pwm_pin=left_motor_forward_pwm_pin, left_motor_backward_pwm_pin=left_motor_backward_pwm_pin,
                                                  right_motor_forward_pwm_pin=right_motor_forward_pwm_pin, right_motor_backward_pwm_pin=right_motor_backward_pwm_pin)
        self.lights_controller = LightsController(np)
        self.display = Display(i2c=i2c)
        self.robot_state_machine = RobotStateMachine(robot=self)
        self.movement_controller = MovementController(
            motors_controller=self.motors_controller, display=self.display, robot_state_machine=self.robot_state_machine)
        self.line_tracking = LineTracking(
            display=self.display, motors_controller=self.motors_controller, movement_controller=self.movement_controller, robot=self)
        self.path = Path(path_actions=path_actions)
        self.adaptive_cruise_control_enabled = False
        self.follow_the_line_enabled = False
        self.speed_regulation_enabled = False

    def regulation_level1(self, timer=None):
        """
        Perform various regulation tasks on Level 1.
        This should be fastest regulation level, typically for speed regulation.
        This is called periodically by the timer.
        """
        # Regulate pwm to achieve desired speed
        if self.speed_regulation_enabled:
            self.motors_controller.speed_regulation(
                self.movement_controller.desired_direction, timer)

    def regulation_level2(self, timer=None):
        """
        Perform various regulation tasks on Level 2.
        This should be 5x slower level, typically for higher level stuff such as cruise control or line tracking.
        This is called periodically by the timer.
        """
        # Regulate speed with adaptive cruise control
        if self.adaptive_cruise_control_enabled:
            self.movement_controller.adaptive_cruise_control()

        # Regulate speed with line tracking
        self.line_tracking.regulation_action()

    def enable_adaptive_cruise_control(self, state: bool):
        self.adaptive_cruise_control_enabled = state

    def enable_follow_the_line(self, state: bool):
        self.follow_the_line_enabled = state

    def enable_speed_regulation(self, state: bool):
        self.speed_regulation_enabled = state

    def is_follow_the_line_enabled(self):
        return self.follow_the_line_enabled
    