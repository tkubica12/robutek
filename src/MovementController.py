from Components import UltrasonicSensor, CollisionSensor, TrackingSensors
from MotorsController import MotorsController, Direction
from Display import Display
from machine import Pin
from machine import I2C
from RobotStateMachineStates import *

class MovementController:
    """
    Class that handles the movement control of the robot.
    This includes speed adjustments such as adaptive cruise control or collision detection.
    TBD: Carrot-chasing algorithm
    TBD: Line-following algorithm
    """
    STOP_DISTANCE = 0.2
    REACTION_POINT = 0.6
    FOLLOW_THE_LINE_MAX_RADIUS = 0.25
    FOLLOW_THE_LINE_MIN_RADIUS = 0.08
    FOLLOW_THE_LINE_CHANGE_RATE = 0.9

    def __init__(self, motors_controller: MotorsController, display: Display, robot_state_machine: RobotStateMachine):
        self.motors_controller = motors_controller
        self.display = display
        self.robot_state_machine = robot_state_machine
        # self.central_ultrasonic_sensor = UltrasonicSensor(trigger_pin=Pin(8, Pin.OUT), echo_pin=Pin(12, Pin.IN))
        self.left_collision_sensor = CollisionSensor(0)
        self.right_collision_sensor = CollisionSensor(0)
        self.adaptive_cruise_control_enabled = False
        self.follow_the_line_enabled = False
        self.speed_regulation_enabled = True
        self.desired_speed = 0
        self.desired_direction = Direction.FORWARD
        self.center_tracking_was_on_line = False
        self.left_tracking_was_on_line = False
        self.right_tracking_was_on_line = False
        self.left_tracking_current = False
        self.center_tracking_current = False
        self.right_tracking_current = False
        self.follow_the_line_current_radius = self.FOLLOW_THE_LINE_MAX_RADIUS
        self.distance_reached_alarm_start = 0
        self.distance_reached_alarm_target = 0
        self.distance_reached_alarm_set = False
        self.turning_angle_reached_alarm_set = False
        self.turning_angle_reached_alarm_start = 0
        self.turning_angle_reached_alarm_target = 0

    def drive_desired_state(self, desired_speed: float, direction: Direction):
        """
        Drive the motors in the desired direction with the desired speed.
        This is called by user.
        """
        self.desired_direction = direction
        self.desired_speed = desired_speed
        self.motors_controller.drive(desired_speed, direction, 0)

    def get_desired_speed(self):
        return self.desired_speed
    
    def get_desired_direction(self):
        return self.desired_direction

    def adaptive_cruise_control(self):
        """
        Adaptive Cruise Control algorithm implementation.
        """
        # Measure distance
        measured_distance = self.central_ultrasonic_sensor.measure_distance()

        # Proportional gain should be scaled from 0 at stop distance to desired speed at reaction point
        proportional_gain = self.desired_speed/(self.REACTION_POINT - self.STOP_DISTANCE)

        # Proportional gain should be scaled from 0 at stop distance to desired speed at zero distance
        proportional_gain_backward = self.desired_speed/self.STOP_DISTANCE

        # Calculate the error and get adjusted speed
        error = self.STOP_DISTANCE - measured_distance
        desired_direction = self.desired_direction
        if error <= 0:
            adjusted_speed = min(-proportional_gain * error, self.desired_speed)
        else:
            adjusted_speed = min(proportional_gain_backward * error, self.desired_speed)
            if desired_direction == Direction.FORWARD:
                desired_direction = Direction.BACKWARD
            else:
                desired_direction = Direction.FORWARD

        # print(f"Measured distance: {measured_distance}, adjusted speed: {adjusted_speed}")

        # Drive the robot with adjusted values (without changing desired speed as decided by user)
        self.motors_controller.drive(adjusted_speed, desired_direction)

    def measurements(self, timer=None):
        """
        Perform regular measurements.
        Generate DistanceReached event when the distance is reached.
        """
        if self.distance_reached_alarm_set:
            # Check if the distance is reached
            if self.motors_controller.left_speed_sensor.radians_counter + self.motors_controller.right_speed_sensor.radians_counter >= self.distance_reached_alarm_target:
                self.robot_state_machine.handle_event(DistanceReached())
                self.distance_reached_alarm_set = False

        if self.turning_angle_reached_alarm_set:
            # Check if the turning angle is reached
            if self.motors_controller.left_speed_sensor.radians_counter >= self.turning_angle_reached_alarm_target:
                self.robot_state_machine.handle_event(TurningAngleReached())
                self.turning_angle_reached_alarm_set = False

    def register_distance_reached_alarm(self, distance: float):
        """
        Fire DistanceReached event when the distance is reached.
        """
        # Set alarm
        self.distance_reached_alarm_set = True

        # Store the current radians counters (we will simply sum from both wheels)
        self.distance_reached_alarm_start = self.motors_controller.left_speed_sensor.radians_counter + self.motors_controller.right_speed_sensor.radians_counter

        # Calculate radians required to travel the distance (we will simply sum from both wheels)
        distance_radians = distance / self.motors_controller.WHEEL_DIAMETER
        self.distance_reached_alarm_target = self.distance_reached_alarm_start + distance_radians*2

    def register_turning_angle_reached_alarm(self, angle: float):
        """
        Fire TurningAngleReached event when the turning angle is reached.
        """
        ANGLE_FOR_RADIAN = 19

        # Set alarm
        self.turning_angle_reached_alarm_set = True

        # Store the current radians counters for one wheel
        self.turning_angle_reached_alarm_start = self.motors_controller.left_speed_sensor.radians_counter

        # Calculate radians required to turn the angle (we will simply sum from both wheels)
        angle_radians = angle / ANGLE_FOR_RADIAN
        self.turning_angle_reached_alarm_target = self.turning_angle_reached_alarm_start + angle_radians