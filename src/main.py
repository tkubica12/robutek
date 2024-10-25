from machine import Pin, Timer, I2C, PWM
from neopixel import NeoPixel
from Robot import Robot
from MotorsController import Direction, TurnDirection
from time import sleep_ms
from RobotStateMachineStates import *
from Path import Path, PathAction
from Configuration import CONFIG

# Planned path actions
path_actions = [
    PathAction.TURN_LEFT, 
    PathAction.GO_STRAIGHT,
    PathAction.TURN_RIGHT, 
    PathAction.TURN_RIGHT, 
    PathAction.GO_STRAIGHT,
    PathAction.GO_STRAIGHT,
    PathAction.TURN_RIGHT, 
    PathAction.TURN_RIGHT, 
    PathAction.GO_STRAIGHT,
    PathAction.GO_STRAIGHT,
    PathAction.TURN_LEFT, 
    PathAction.TURN_LEFT, 
    # PathAction.GO_STRAIGHT,
    # PathAction.GO_STRAIGHT,
    # PathAction.GO_STRAIGHT,
    # PathAction.TURN_LEFT, 
    ]

# Create instance of Robot
r = Robot(
    np = NeoPixel(Pin(14),8),
    i2c = I2C(0, sda=Pin(19), scl=Pin(18), freq=400000),
    left_motor_forward_pwm_pin = PWM(Pin(26), freq=2000),
    left_motor_backward_pwm_pin = PWM(Pin(27), freq=2000),
    right_motor_forward_pwm_pin = PWM(Pin(25), freq=2000),
    right_motor_backward_pwm_pin = PWM(Pin(33), freq=2000),
    path_actions = path_actions
)

# Register timer callback for blinking lights
movement_timer_blinker = Timer(0)
movement_timer_blinker.init(mode=Timer.PERIODIC, period=500, callback=r.lights_controller.process)

# Register timer for level 1 regulation
# movement_timer_level1 = Timer(0)
# movement_timer_level1.init(mode=Timer.PERIODIC, period=50, callback=r.regulation_level1)  # This is just due to encoder issues, should be much smaller

# Register timer for level 2 regulation
movement_timer_level2 = Timer(1)
movement_timer_level2.init(mode=Timer.PERIODIC, period=2, callback=r.regulation_level2)

# Register timer for measurements
measurement_timer = Timer(2)
measurement_timer.init(mode=Timer.PERIODIC, period=5, callback=r.movement_controller.measurements)

# Register callbacks for speed sensors
left_speed_sensor = Pin(32, Pin.IN, Pin.PULL_DOWN)
right_speed_sensor = Pin(35, Pin.IN, Pin.PULL_DOWN)
left_speed_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=r.motors_controller.left_speed_sensor.process_event)
right_speed_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=r.motors_controller.right_speed_sensor.process_event)

# Features configuration
r.enable_adaptive_cruise_control(False)
r.enable_follow_the_line(False)
r.enable_speed_regulation(False)

sleep_ms(5000)

r.robot_state_machine.handle_event(StartMoving(CONFIG["DEMO_FORWARD_SPEED"], Direction.FORWARD))

