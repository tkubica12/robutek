from Display import Display
from MotorsController import MotorsController
from MovementController import MovementController
from time import ticks_ms, ticks_diff, sleep_ms
from RobotStateMachineStates import *
from RobotStateMachineEvents import *
from MotorsController import Direction
from machine import Pin
from Configuration import CONFIG


class LineTracking:
    def __init__(self, display: Display, motors_controller: MotorsController, movement_controller: MovementController, robot):
        self.follow_the_line_angular_velocity = 0
        self.display = display
        self.motors_controller = motors_controller
        self.movement_controller = movement_controller
        self.robot = robot
        self.left_tracking_sensor_current = False
        self.center_left_tracking_sensor_current = False
        self.center_right_tracking_sensor_current = False
        self.right_tracking_sensor_current = False
        self.left_sensor_last_change = 0
        self.center_left_sensor_last_change = 0
        self.center_right_sensor_last_change = 0
        self.right_sensor_last_change = 0
        self.is_regulating = False
        self.left_tracking_sensor = Pin(21, Pin.IN, Pin.PULL_DOWN)
        self.center_left_tracking_sensor = Pin(22, Pin.IN, Pin.PULL_DOWN)
        self.center_right_tracking_sensor = Pin(23, Pin.IN, Pin.PULL_DOWN)
        self.right_tracking_sensor = Pin(5, Pin.IN, Pin.PULL_DOWN)

    def set_angular_velocity(self, angular_velocity):
        """
        Set the angular velocity.
        """
        self.follow_the_line_angular_velocity = angular_velocity
        self.follow_the_line_drive()

    def start_regulation(self):
        """
        Start the line tracking regulation.
        """
        self.is_regulating = True
        self.regulation_start_time = ticks_ms()

    def stop_regulation(self):
        """
        Stop the line tracking regulation.
        """
        self.is_regulating = False

    def regulation_action(self):
        """
        Perform the action when correction is required.
        """
        # Read line tracking sensors
        left = self.left_tracking_sensor.value()
        center_left = self.center_left_tracking_sensor.value()
        center_right = self.center_right_tracking_sensor.value()
        right = self.right_tracking_sensor.value()

        # Emit events when needed
        if left != self.left_tracking_sensor_current:
            self.robot.robot_state_machine.handle_event(LeftDetected() if left else LeftLost())
        if right != self.right_tracking_sensor_current:
            self.robot.robot_state_machine.handle_event(RightDetected() if right else RightLost())
        if (center_left or center_right) != (self.center_left_tracking_sensor_current or self.center_right_tracking_sensor_current):
            self.robot.robot_state_machine.handle_event(CenterDetected() if center_left or center_right else CenterLost())

        # Save the current state of the sensors
        self.left_tracking_sensor_current = left
        self.center_left_tracking_sensor_current = center_left
        self.center_right_tracking_sensor_current = center_right
        self.right_tracking_sensor_current = right

        # Correct the angular velocity if needed
        if self.robot.is_follow_the_line_enabled():
            self.angular_velocity_correction()

        # Run cross detection
        self.cross_detection()

        # if self.is_regulating:
        #     current_time = ticks_ms()
        #     elapsed_time = ticks_diff(current_time, self.regulation_start_time)
        #     regulation_correction = CONFIG["LINE_TRACKING_P"] * elapsed_time
        #     if self.follow_the_line_angular_velocity > 0:
        #         self.follow_the_line_angular_velocity += regulation_correction
        #         self.follow_the_line_angular_velocity = min(
        #             self.follow_the_line_angular_velocity, CONFIG["MAX_ANGULAR_VELOCITY"])
        #     else:
        #         self.follow_the_line_angular_velocity -= regulation_correction
        #         self.follow_the_line_angular_velocity = max(
        #             self.follow_the_line_angular_velocity, -CONFIG["MAX_ANGULAR_VELOCITY"])
        #     self.follow_the_line_drive()

    def angular_velocity_correction(self):
        # Both center sensors see the line, no correction needed
        if self.center_left_tracking_sensor_current and self.center_right_tracking_sensor_current:
            self.set_angular_velocity(0)
        # Only the left center sensor sees the line, turn left
        elif self.center_left_tracking_sensor_current:
            self.set_angular_velocity(-CONFIG["CENTER_CORRECTION_ANGULAR_VELOCITY"])
        # Only the right center sensor sees the line, turn right
        elif self.center_right_tracking_sensor_current:
            self.set_angular_velocity(CONFIG["CENTER_CORRECTION_ANGULAR_VELOCITY"])
        # Outer left sensor sees the line, turn left more aggressively
        elif self.left_tracking_sensor_current:
            self.set_angular_velocity(-CONFIG["INITIAL_ANGULAR_VELOCITY"])
        # Outer right sensor sees the line, turn right more aggressively
        elif self.right_tracking_sensor_current:
            self.set_angular_velocity(CONFIG["INITIAL_ANGULAR_VELOCITY"])

        self.follow_the_line_drive()

    def follow_the_line_drive(self):
        """
        Drive the robot following the line.
        """
        self.motors_controller.drive(
            speed=self.movement_controller.get_desired_speed(),
            direction=self.movement_controller.get_desired_direction(),
            angular_velocity=self.follow_the_line_angular_velocity
        )

    def stop(self):
        """
        Stop the robot.
        """
        self.motors_controller.drive(
            speed=0,
            direction=self.movement_controller.get_desired_direction(),
            angular_velocity=0
        )

    def is_on_cross(self):
        """
        Check if the robot is on a cross.
        For purpose of cross detection two central sensors are combined to one.
        Two checks are performed:
            1. At least two sensors are detecting the line at the same time.
            2. All three sensors are detecting the line within a correlation window.
        """
        current_time = ticks_ms()

        # Left with Center Right
        left_with_center_right = self.left_tracking_sensor_current and self.center_right_tracking_sensor_current

        # Right with Center Left
        right_with_center_left = self.right_tracking_sensor_current and self.center_left_tracking_sensor_current

        # Left with Right
        left_with_right = self.left_tracking_sensor_current and self.right_tracking_sensor_current

        # All sensors within correlation window
        all_sensors_with_correlation = sum([
             self.left_tracking_sensor_current or ticks_diff(current_time, self.left_sensor_last_change) < CONFIG["CROSS_DETECTION_CORRELATION_WINDOW"],
                self.center_left_tracking_sensor_current or ticks_diff(current_time, self.center_left_sensor_last_change) < CONFIG["CROSS_DETECTION_CORRELATION_WINDOW"],
                self.center_right_tracking_sensor_current or ticks_diff(current_time, self.center_right_sensor_last_change) < CONFIG["CROSS_DETECTION_CORRELATION_WINDOW"],
                self.right_tracking_sensor_current or ticks_diff(current_time, self.right_sensor_last_change) < CONFIG["CROSS_DETECTION_CORRELATION_WINDOW"]
        ]) >= 4

        return left_with_center_right or right_with_center_left or left_with_right or all_sensors_with_correlation

    def cross_detection(self):
        """
        Check sensor values for cross detection.
        """
        if self.is_on_cross():
            self.robot.robot_state_machine.handle_event(CrossDetected())

    def drive_back_to_cross(self):
        """
        Drive the robot slowly backward so we can detect the cross we have overshoot.
        """
        self.movement_controller.drive_desired_state(
            desired_speed=CONFIG["CENTERING_SPEED"], direction=Direction.BACKWARD)

    def center_to_cross(self):
        """
        Center the robot on the cross.
        """
        self.movement_controller.register_distance_reached_alarm(CONFIG["DISTANCE_SENSORS_TO_CENTER"])
        self.movement_controller.drive_desired_state(desired_speed=CONFIG["CENTERING_SPEED"], direction=Direction.FORWARD)

    # def left_sensor_to_event(self, pin):
    #     """
    #     Convert left sensor interrupt to event.
    #     """
    #     if self.left_tracking_sensor_current != pin.value():
    #         # Save the current state of the sensor
    #         self.left_tracking_sensor_current = pin.value()
    #         self.left_sensor_last_change = ticks_ms()

    #         # Detect cross
    #         self.cross_detection()

    #         # Generate event for the state machine
    #         if self.left_tracking_sensor_current:
    #             self.robot.robot_state_machine.handle_event(LeftDetected())
    #         else:
    #             self.robot.robot_state_machine.handle_event(LeftLost())

    # def center_left_sensor_to_event(self, pin):
    #     """
    #     Convert center sensor interrupt to event.
    #     """
    #     if self.center_left_tracking_sensor_current != pin.value():
    #         # Save the current state of the sensor
    #         self.center_left_tracking_sensor_current = pin.value()
    #         self.center_left_sensor_last_change = ticks_ms()

    #         # Detect cross
    #         self.cross_detection()

    #         # Generate event for the state machine
    #         if self.center_left_tracking_sensor_current:
    #             self.robot.robot_state_machine.handle_event(CenterLeftDetected())
    #         else:
    #             self.robot.robot_state_machine.handle_event(CenterLeftLost())

    # def center_right_sensor_to_event(self, pin):
    #     """
    #     Convert center sensor interrupt to event.
    #     """
    #     if self.center_right_tracking_sensor_current != pin.value():
    #         # Save the current state of the sensor
    #         self.center_right_tracking_sensor_current = pin.value()
    #         self.center_right_sensor_last_change = ticks_ms()

    #         # Detect cross
    #         self.cross_detection()

    #         # Generate event for the state machine
    #         if self.center_right_tracking_sensor_current:
    #             self.robot.robot_state_machine.handle_event(CenterRightDetected())
    #         else:
    #             self.robot.robot_state_machine.handle_event(CenterRightLost())

    # def right_sensor_to_event(self, pin):
    #     """
    #     Convert right sensor interrupt to event.
    #     """
    #     if self.left_tracking_sensor_current != pin.value():
    #         # Save the current state of the sensor
    #         self.right_tracking_sensor_current = pin.value()
    #         self.right_sensor_last_change = ticks_ms()

    #         # Detect cross
    #         self.cross_detection()

    #         # Generate event for the state machine
    #         if self.right_tracking_sensor_current:
    #             self.robot.robot_state_machine.handle_event(RightDetected())
    #         else:
    #             self.robot.robot_state_machine.handle_event(RightLost())

    # def center_line_correction(self):
    #     """
    #     Perform the actions when the center line is lost.
    #     """
    #     # Both center sensors see the line, no correction needed
    #     if self.center_left_tracking_sensor_current and self.center_right_tracking_sensor_current:
    #         self.set_angular_velocity(0)
    #     # Only the left center sensor sees the line, turn left
    #     elif self.center_left_tracking_sensor_current:
    #         self.set_angular_velocity(-CONFIG["CENTER_CORRECTION_ANGULAR_VELOCITY"])
    #     # Only the right center sensor sees the line, turn right
    #     elif self.center_right_tracking_sensor_current:
    #         self.set_angular_velocity(CONFIG["CENTER_CORRECTION_ANGULAR_VELOCITY"])
    #     # No center sensor sees the line, no action, different state will handle this
    #     return