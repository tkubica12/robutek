from MotorsController import Direction, TurnDirection
from time import sleep_ms
from RobotStateMachineEvents import *
from machine import Timer
from Path import Path, PathAction
from Configuration import CONFIG

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
    """
    Waiting for the start moving event.
    """
    _instance = None

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(Idle, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Entering Idle State")
        self.robot.display.show_message("Idle", 0)

    def on_exit(self):
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Exiting Idle State")

    def handle_event(self, event):
        if isinstance(event, StartMoving):
            print(f"Moving at {event.speed} speed")
            self.robot.movement_controller.drive_desired_state(event.speed, event.direction)
            return FollowTheLine(self.robot)
        return self

class FollowTheLine(State):
    """
    Following the line and moving forward.
    """
    _instance = None

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(FollowTheLine, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Entering Follow The Line State")
        self.robot.display.show_message("Follow line", 0)
        self.robot.lights_controller.main_on()
        self.robot.enable_follow_the_line(True)
        self.robot.enable_speed_regulation(False)
        self.robot.line_tracking.display.show_message("In Center", 2)
        self.robot.line_tracking.set_angular_velocity(0)
        self.robot.line_tracking.stop_regulation()
        self.robot.line_tracking.follow_the_line_drive()

    def on_exit(self):
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Exiting Follow The Line State")

    def handle_event(self, event):
        if isinstance(event, CrossDetected):
            return CrossStopping(self.robot)
        elif isinstance(event, Stop):
            return Idle(self.robot)
        elif isinstance(event, StartMoving):
            self.robot.movement_controller.drive_desired_state(event.speed, event.direction)
            return self
        elif isinstance(event, DistanceReached):
            # Pass this event to LineTrackingStateMachine
            self.robot.robot_state_machine.handle_event(event)
            return self
        return self

class HandleCrossing(State):
    """
    Handling the crossing such as turning left, right or going straight/
    """
    _instance = None

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(HandleCrossing, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Entering Handle Crossing State")
        self.line_detections_enabled = False
        self.robot.display.show_message("Handle crossing", 0)
        self.robot.enable_follow_the_line(False)

        # Decide based on next action in the list
        next_action = self.robot.path.next_action()
        if next_action == PathAction.TURN_LEFT:
            self.robot.lights_controller.left_blinker_on()
            self.turn_direction = TurnDirection.COUNTER_CLOCKWISE
            self.robot.display.show_message("Turning left", 1)
            self.start_turning(self.turn_direction)
        elif next_action == PathAction.TURN_RIGHT:
            self.robot.lights_controller.right_blinker_on()
            self.turn_direction = TurnDirection.CLOCKWISE
            self.robot.display.show_message("Turning right", 1)
            self.start_turning(self.turn_direction)
        elif next_action == PathAction.GO_STRAIGHT:
            self.robot.display.show_message("Going straight", 1)
            self.move_over_cross()
        else:  # No more actions left, stop
            self.robot.robot_state_machine.handle_event(NoPathActionsLeft())

    def on_exit(self):
        self.robot.lights_controller.left_blinker_off()
        self.robot.lights_controller.right_blinker_off()
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Exiting Handle Crossing State")

    def start_turning(self, turn_direction):
        self.robot.movement_controller.register_turning_angle_reached_alarm(30)
        self.robot.motors_controller.stationary_turn(CONFIG["TURNING_INITIAL_SPEED"], turn_direction)

    def move_over_cross(self):
        self.movement_controller.register_distance_reached_alarm(CONFIG["MOVE_OVER_CROSS_DISTANCE"])
        self.movement_controller.drive_desired_state(desired_speed=CONFIG["MOVE_OVER_CROSS_SPEED"], direction=Direction.FORWARD)

    def handle_event(self, event):
        if isinstance(event, TurningAngleReached):  # Initial turning finished, slow down, start detecting lines
            self.line_detections_enabled = True
            self.robot.motors_controller.stationary_turn(CONFIG["TURNING_NORMAL_SPEED"], self.turn_direction)
            return self
        elif isinstance(event, LeftDetected) or isinstance(event, RightDetected):   # Left or Right line detected, significantly slow down
            if self.line_detections_enabled:
                self.robot.motors_controller.stationary_turn(CONFIG["TURNING_REDUCED_SPEED"], self.turn_direction)
            return self
        elif isinstance(event, CenterDetected):  # Center line detected, turning finished
            if self.line_detections_enabled:
                self.robot.display.show_message("", 1)
                self.robot.movement_controller.drive_desired_state(CONFIG["DEMO_FORWARD_SPEED"], Direction.FORWARD)
                return FollowTheLine(self.robot)
            else:
                return self
        elif isinstance(event, DistanceReached):  # Distance reached, moved over the cross
            self.robot.display.show_message("", 1)
            self.robot.movement_controller.drive_desired_state(CONFIG["DEMO_FORWARD_SPEED"], Direction.FORWARD)
            return FollowTheLine(self.robot)
        return self

class EmergencyState(State):
    """
    Emergency state when something goes wrong.
    """
    _instance = None

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(EmergencyState, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        self.robot.lights_controller.hazard_on()
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Entering Emergency State")
        self.robot.movement_controller.drive_desired_state(0, Direction.FORWARD)

    def handle_event(self, event):
        # Stay in emergency state until reset or resolved
        return self
    
class CrossCentering(State):
    """
    Centering the robot on the cross.
    """
    def on_enter(self):
        if CONFIG["LOG_LEVEL"] >= 1:
            print("Entering CrossCentering State")
        # self.robot.line_tracking.center_to_cross()
        if self.robot.line_tracking.is_on_cross():
            self.robot.line_tracking.display.show_message("Cross Centering", 2)
            self.robot.lights_controller.main_on()
            self.robot.line_tracking.center_to_cross()
        else:
            self.robot.line_tracking.display.show_message("Cross overshoot", 2)
            self.robot.lights_controller.reverse_on()
            self.robot.line_tracking.drive_back_to_cross()

    def on_exit(self):
        if CONFIG["LOG_LEVEL"] >= 1:
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
    """
    Stopping the robot on the cross.
    """
    def on_enter(self):
        self.robot.lights_controller.break_on()
        if CONFIG["LOG_LEVEL"] >= 1:
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
        self.robot.lights_controller.break_off()
        if CONFIG["LOG_LEVEL"] >= 1:
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
        if CONFIG["LOG_LEVEL"] >= 1:
            print(f"Handling event of type: {type(event).__name__}")
        if isinstance(event, EmergencyEvent):
            new_state = EmergencyState(self.robot)
        else:
            new_state = self.state.handle_event(event)
        
        if new_state != self.state:
            self.state.on_exit()
            self.state = new_state
            self.state.on_enter()
