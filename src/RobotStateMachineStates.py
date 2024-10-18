from MotorsController import Direction, TurnDirection
from time import sleep_ms
from RobotStateMachineEvents import *
from machine import Timer

# Define States
class State:
    def __init__(self, robot):
        self.robot = robot

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def handle_event(self, event):
        pass

class Idle(State):
    _instance = None

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(Idle, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        print("Entering Idle State")
        self.robot.display.show_message("Idle", 0)

    def on_exit(self):
        print("Exiting Idle State")

    def handle_event(self, event):
        if isinstance(event, StartMoving):
            print(f"Moving at {event.speed} speed")
            self.robot.movement_controller.drive_desired_state(event.speed, event.direction)
            return FollowTheLine(self.robot)
        return self

class FollowTheLine(State):
    _instance = None

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(FollowTheLine, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        print("Entering Follow The Line State")
        self.robot.display.show_message("Follow line", 0)
        self.robot.enable_follow_the_line(True)
        self.robot.enable_speed_regulation(False)
        self.robot.line_tracking.display.show_message("In Center", 2)
        self.robot.line_tracking.set_angular_velocity(0)
        self.robot.line_tracking.stop_regulation()
        self.robot.line_tracking.follow_the_line_drive()

    def on_exit(self):
        print("Exiting Follow The Line State")

    def handle_event(self, event):
        if isinstance(event, LeftDetected):
            return CorrectionLeft(self.robot)
        elif isinstance(event, RightDetected):
            return CorrectionRight(self.robot)
        elif isinstance(event, CrossDetected):
            return CrossStopping(self.robot)
        elif isinstance(event, Stop):
            return Idle(self.robot)
        elif isinstance(event, OnCross):
            return HandleCrossing(self.robot)
        elif isinstance(event, StartMoving):
            self.robot.movement_controller.drive_desired_state(event.speed, event.direction)
            return self
        elif isinstance(event, DistanceReached):
            # Pass this event to LineTrackingStateMachine
            self.robot.robot_state_machine.handle_event(event)
            return self
        return self

class HandleCrossing(State):
    _instance = None
    DISTANCE_SENSORS_TO_CENTER = 0.03

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(HandleCrossing, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        print("Entering Handle Crossing State")
        self.line_detections_enabled = False
        self.robot.display.show_message("Handle crossing", 0)
        self.robot.enable_follow_the_line(False)
        self.robot.display.show_message("Turning", 1)
        self.robot.movement_controller.register_turning_angle_reached_alarm(45)
        self.robot.motors_controller.stationary_turn(0.6, TurnDirection.COUNTER_CLOCKWISE)

    def on_exit(self):
        print("Exiting Handle Crossing State")

    def handle_event(self, event):
        if isinstance(event, TurningAngleReached):
            self.line_detections_enabled = True
            self.robot.motors_controller.stationary_turn(0.4, TurnDirection.COUNTER_CLOCKWISE)
            return self
        elif isinstance(event, LeftDetected):
            if self.line_detections_enabled:
                self.robot.motors_controller.stationary_turn(0.25, TurnDirection.COUNTER_CLOCKWISE)
            return self
        elif isinstance(event, CenterDetected):
            if self.line_detections_enabled:
                self.robot.display.show_message("", 1)
                self.robot.movement_controller.drive_desired_state(0.43, Direction.FORWARD)
                return FollowTheLine(self.robot)
            else:
                return self
        return self

class EmergencyState(State):
    _instance = None

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(EmergencyState, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        print("Entering Emergency State")
        self.robot.stop()

    def handle_event(self, event):
        # Stay in emergency state until reset or resolved
        return self
    
class LineTrackingState:
    def __init__(self, line_tracking):
        self.robot.line_tracking = line_tracking

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def handle_event(self, event):
        pass

class CorrectionLeft(State):
    def on_enter(self):
        print("Entering CorrectionLeft State")
        self.robot.line_tracking.display.show_message("Correction Left", 2)
        self.robot.line_tracking.set_angular_velocity(-1.0)
        self.robot.line_tracking.start_regulation()
        self.robot.line_tracking.correction_action()

    def on_exit(self):
        print("Exiting CorrectionLeft State")
        self.robot.line_tracking.display.show_message("", 2)

    def handle_event(self, event):
        if isinstance(event, CenterDetected):
            return FollowTheLine(self.robot)
        elif isinstance(event, RightDetected):
            return CorrectionRight(self.robot)
        elif isinstance(event, CrossDetected):
            return CrossStopping(self.robot)
        return self

class CorrectionRight(State):
    def on_enter(self):
        print("Entering CorrectionRight State")
        self.robot.line_tracking.display.show_message("Correction Right", 2)
        self.robot.line_tracking.set_angular_velocity(1.0)
        self.robot.line_tracking.start_regulation()
        self.robot.line_tracking.correction_action()

    def on_exit(self):
        print("Exiting CorrectionRight State")
        self.robot.line_tracking.display.show_message("", 2)

    def handle_event(self, event):
        if isinstance(event, CenterDetected):
            return FollowTheLine(self.robot)
        elif isinstance(event, LeftDetected):
            return CorrectionLeft(self.robot)
        elif isinstance(event, CrossDetected):
            return CrossStopping(self.robot)
        return self
    
class CrossCentering(State):
    def on_enter(self):
        print("Entering CrossCentering State")
        self.robot.line_tracking.center_to_cross()
        # if self.robot.line_tracking.is_on_cross():
        #     self.robot.line_tracking.display.show_message("Cross Centering", 2)
        #     self.robot.line_tracking.center_to_cross()
        # else:
        #     self.robot.line_tracking.display.show_message("Cross overshoot", 2)
        #     self.robot.line_tracking.drive_back_to_cross()

    def on_exit(self):
        print("Exiting CrossCentering State")
        self.robot.line_tracking.display.show_message("", 2)

    def handle_event(self, event):
        if isinstance(event, CrossDetected):
            self.on_enter()
        if isinstance(event, DistanceReached):
            self.robot.line_tracking.movement_controller.drive_desired_state(0, Direction.FORWARD)
            return HandleCrossing(self.robot)
        return self

class CrossStopping(State):
    def on_enter(self):
        print("Entering CrossStopping State")
        self.robot.line_tracking.stop()
        self.robot.line_tracking.display.show_message("Cross Stopping", 2)
        self.robot.line_tracking.set_angular_velocity(0)
        self.robot.line_tracking.stop_regulation()
        self.robot.line_tracking.robot.enable_follow_the_line(False)

        # Set up a timer to handle the delay non-blocking
        self.timer = Timer(3)
        self.timer.init(period=200, mode=Timer.ONE_SHOT, callback=self.on_timer_complete)

    def on_timer_complete(self, t):
        self.robot.robot_state_machine.handle_event(Stopped())

    def on_exit(self):
        print("Exiting CrossStopping State")
        self.robot.line_tracking.display.show_message("", 2)
        # Deinitialize the timer if it exists
        if hasattr(self, 'timer'):
            self.timer.deinit()

    def handle_event(self, event):
        if isinstance(event, Stopped):
            return CrossCentering(self.robot)
        return self


# Define State Machine
class RobotStateMachine:
    def __init__(self, robot):
        self.robot = robot
        self.state = Idle(robot)
        self.state.on_enter()

    def handle_event(self, event):
        print(f"Handling event of type: {type(event).__name__}")
        if isinstance(event, EmergencyEvent):
            new_state = EmergencyState(self.robot)
        else:
            new_state = self.state.handle_event(event)
        
        if new_state != self.state:
            self.state.on_exit()
            self.state = new_state
            self.state.on_enter()
