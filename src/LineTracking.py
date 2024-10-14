from LineTrackingStateMachine import (
    LineTrackingStateMachine,
    InCenter,
    CorrectionLeft,
    CorrectionRight,
    CenterDetected,
    LeftDetected,
    RightDetected,
    CenterLost,
    LeftLost,
    RightLost,
    CrossDetected,
)

from Display import Display
from MotorsController import MotorsController
from MovementController import MovementController
from time import ticks_ms, ticks_diff, sleep_ms
from RobotStateMachine import OnCross
from MotorsController import Direction
from machine import Pin

class LineTracking:
    P = 0.001
    MAX_ANGULAR_VELOCITY = 2.5
    CROSS_DETECTION_CORRELATION_WINDOW = 50
    BACKWARD_SPEED = 0.15

    def __init__(self, display: Display, motors_controller: MotorsController, movement_controller: MovementController, robot):
        self.follow_the_line_angular_velocity = 0
        self.display = display
        self.motors_controller = motors_controller
        self.movement_controller = movement_controller
        self.robot = robot
        self.line_tracking_state_machine = LineTrackingStateMachine(self)
        self.left_tracking_sensor_current = False
        self.center_tracking_sensor_current = False
        self.right_tracking_sensor_current = False
        self.left_sensor_last_change = 0
        self.center_sensor_last_change = 0
        self.right_sensor_last_change = 0

    def set_angular_velocity(self, angular_velocity):
        """
        Set the angular velocity.
        """
        self.follow_the_line_angular_velocity = angular_velocity

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

    def correction_action(self):
        """
        Perform the action when correction is required.
        """
        if self.is_regulating:
            current_time = ticks_ms()
            elapsed_time = ticks_diff(current_time, self.regulation_start_time)
            regulation_correction = self.P * elapsed_time
            if self.follow_the_line_angular_velocity > 0:
                self.follow_the_line_angular_velocity += regulation_correction
                self.follow_the_line_angular_velocity = min(self.follow_the_line_angular_velocity, self.MAX_ANGULAR_VELOCITY)
            else:
                self.follow_the_line_angular_velocity -= regulation_correction
                self.follow_the_line_angular_velocity = max(self.follow_the_line_angular_velocity, -self.MAX_ANGULAR_VELOCITY)
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
        We calculate either sensors that currently see the line or went off within the correlation window.
        At this point I keep it as 3 to detect crossing - this cannot handle T-crossing yet.
        """
        sensors_detecting = sum([
            self.left_tracking_sensor_current or ticks_diff(ticks_ms(), self.left_sensor_last_change) < self.CROSS_DETECTION_CORRELATION_WINDOW,
            self.center_tracking_sensor_current or ticks_diff(ticks_ms(), self.center_sensor_last_change) < self.CROSS_DETECTION_CORRELATION_WINDOW,
            self.right_tracking_sensor_current or ticks_diff(ticks_ms(), self.right_sensor_last_change) < self.CROSS_DETECTION_CORRELATION_WINDOW
        ])

        if sensors_detecting >= 3:
            print(f"Left: {self.left_tracking_sensor_current} time diff: {ticks_diff(ticks_ms(), self.left_sensor_last_change)}")
            print(f"Center: {self.center_tracking_sensor_current} time diff: {ticks_diff(ticks_ms(), self.center_sensor_last_change)}")
            print(f"Right: {self.right_tracking_sensor_current} time diff: {ticks_diff(ticks_ms(), self.right_sensor_last_change)}")
            print(f"Sum: {sum([self.left_tracking_sensor_current or ticks_diff(ticks_ms(), self.left_sensor_last_change) < self.CROSS_DETECTION_CORRELATION_WINDOW, self.center_tracking_sensor_current or ticks_diff(ticks_ms(), self.center_sensor_last_change) < self.CROSS_DETECTION_CORRELATION_WINDOW, self.right_tracking_sensor_current or ticks_diff(ticks_ms(), self.right_sensor_last_change) < self.CROSS_DETECTION_CORRELATION_WINDOW])}")

        return sensors_detecting >= 3

    def cross_detection(self):
        """
        Check sensor values for cross detection.
        """
        if self.is_on_cross():
            self.line_tracking_state_machine.handle_event(CrossDetected())

    def drive_back_to_cross(self):
        """
        Drive the robot slowly in opposite direction so we can detect the cross we have overshoot.
        """
        if self.movement_controller.get_desired_direction() == Direction.FORWARD:
            direction=Direction.BACKWARD
        else:
            direction=Direction.FORWARD

        self.movement_controller.drive_desired_state(desired_speed=self.BACKWARD_SPEED, direction=direction)

    def left_sensor_to_event(self, pin):
        """
        Convert left sensor interrupt to event.
        """
        if self.left_tracking_sensor_current != pin.value():
            # Save the current state of the sensor
            self.left_tracking_sensor_current = pin.value()
            self.left_sensor_last_change = ticks_ms()

            # Detect cross
            self.cross_detection()

            # Generate event for the state machine
            if self.robot.is_follow_the_line_enabled():
                if self.left_tracking_sensor_current:
                    self.line_tracking_state_machine.handle_event(LeftDetected())
                else:
                    self.line_tracking_state_machine.handle_event(LeftLost())

    def center_sensor_to_event(self, pin):
        """
        Convert center sensor interrupt to event.
        """
        if self.left_tracking_sensor_current != pin.value():
            # Save the current state of the sensor
            self.center_tracking_sensor_current = pin.value()
            self.center_sensor_last_change = ticks_ms()

            # Detect cross
            self.cross_detection()

            # Generate event for the state machine
            if self.robot.is_follow_the_line_enabled():
                if self.center_tracking_sensor_current:
                    self.line_tracking_state_machine.handle_event(CenterDetected())
                else:
                    self.line_tracking_state_machine.handle_event(CenterLost())

    def right_sensor_to_event(self, pin):
        """
        Convert right sensor interrupt to event.
        """
        if self.left_tracking_sensor_current != pin.value():
            # Save the current state of the sensor
            self.right_tracking_sensor_current = pin.value()
            self.right_sensor_last_change = ticks_ms()

            # Detect cross
            self.cross_detection()

            # Generate event for the state machine
            if self.robot.is_follow_the_line_enabled():
                if self.right_tracking_sensor_current:
                    self.line_tracking_state_machine.handle_event(RightDetected())
                else:
                    self.line_tracking_state_machine.handle_event(RightLost())