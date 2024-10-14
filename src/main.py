from machine import Pin, Timer, I2C, PWM
from neopixel import NeoPixel
from Robot import Robot
from MotorsController import Direction, TurnDirection
from time import sleep_ms
from RobotStateMachine import StartMoving, Stop, OnCross, TurningFinished, CollisionDetected, AllLinesLost

# Create instance of Robot
r = Robot(
    np = NeoPixel(Pin(23),8),
    i2c = I2C(0, sda=Pin(19), scl=Pin(18), freq=400000),
    left_motor_forward_pwm_pin = PWM(Pin(26), freq=2000),
    left_motor_backward_pwm_pin = PWM(Pin(27), freq=2000),
    right_motor_forward_pwm_pin = PWM(Pin(25), freq=2000),
    right_motor_backward_pwm_pin = PWM(Pin(33), freq=2000)
)

# Register timer callback for blinking lights
# blinker = Timer(mode=Timer.PERIODIC, period=500, callback=r.lights_controller.process)

# Register timer for level 1 regulation
movement_timer_level1 = Timer(0)
movement_timer_level1.init(mode=Timer.PERIODIC, period=50, callback=r.regulation_level1)  # This is just due to encoder issues, should be much smaller

# Register timer for level 2 regulation
movement_timer_level2 = Timer(1)
movement_timer_level2.init(mode=Timer.PERIODIC, period=10, callback=r.regulation_level2)

# Register timer for measurements
measurement_timer = Timer(2)
measurement_timer.init(mode=Timer.PERIODIC, period=5, callback=r.movement_controller.measurements)

# Register callbacks for speed sensors
left_speed_sensor = Pin(32, Pin.IN, Pin.PULL_DOWN)
right_speed_sensor = Pin(35, Pin.IN, Pin.PULL_DOWN)
left_speed_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=r.motors_controller.left_speed_sensor.process_event)
right_speed_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=r.motors_controller.right_speed_sensor.process_event)

# Register callbacks for line sensors
left_tracking_sensor = Pin(21, Pin.IN, Pin.PULL_DOWN)
center_tracking_sensor = Pin(22, Pin.IN, Pin.PULL_DOWN)
right_tracking_sensor = Pin(23, Pin.IN, Pin.PULL_DOWN)
left_tracking_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=r.line_tracking.left_sensor_to_event)
center_tracking_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=r.line_tracking.center_sensor_to_event)
right_tracking_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=r.line_tracking.right_sensor_to_event)

# Features configuration
r.enable_adaptive_cruise_control(False)
# r.enable_follow_the_line(False)
# r.enable_speed_regulation(True)

sleep_ms(5000)

r.robot_state_machine.handle_event(StartMoving(0.30, Direction.FORWARD))

# r.movement_controller.motors_controller.stationary_turn(0.5, TurnDirection.CLOCKWISE)
# r.movement_controller.drive_desired_state(0.25, Direction.FORWARD)
# r.motors_controller.calibrate_advanced(r.motors_controller.left_motor, r.motors_controller.left_speed_sensor, "left_calibration.csv")
# r.motors_controller.stop()
# r.motors_controller.calibrate(r.motors_controller.left_motor, r.motors_controller.left_speed_sensor)
# r.motors_controller.left_motor.forward(40000)
# r.motors_controller.drive(0.2, Direction.FORWARD)
# r.motors_controller.drive(0.2, Direction.BACKWARD)
# r.movement_controller.drive_desired_state(0.15, Direction.BACKWARD)

