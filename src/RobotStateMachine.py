from MotorsController import Direction
from time import sleep_ms

# Define Events
class Event:
    """
    Base class for all events.
    """
    pass

class OnCross(Event):
    """
    Event is triggered when algorithm detects crossing of lines.
    """
    pass

class StartMoving(Event):
    """
    Event is triggered when robot should start moving in forward or backward direction.
    """
    def __init__(self, speed: float, direction: Direction):
        self.speed = speed
        self.direction = direction

class Stop(Event):
    """
    Event is triggered when robot should stop.
    """
    pass

class TurningFinished(Event):
    """
    Event is triggered when robot finishes turning
    """
    pass

class EmergencyEvent(Event):
    """
    Base class for all emergency events.
    """
    pass

class CollisionDetected(EmergencyEvent):
    pass

class AllLinesLost(EmergencyEvent):
    pass

class DistanceReached(Event):
    """
    You can register alert so this event is triggered when set distance is reached.
    This can be used for example to stop the robot after reaching certain distance, 
    eg. to travel distance between line sensors and center of wheels.
    """
    pass

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
        self.robot.enable_speed_regulation(True)

    def on_exit(self):
        print("Exiting Follow The Line State")

    def handle_event(self, event):
        if isinstance(event, Stop):
            return Idle(self.robot)
        elif isinstance(event, OnCross):
            return HandleCrossing(self.robot)
        elif isinstance(event, StartMoving):
            self.robot.movement_controller.drive_desired_state(event.speed, event.direction)
            return self
        return self

class HandleCrossing(State):
    _instance = None
    DISTANCE_SENSORS_TO_CENTER = 0.05

    def __new__(cls, robot):
        if cls._instance is None:
            cls._instance = super(HandleCrossing, cls).__new__(cls)
            cls._instance.robot = robot
        return cls._instance

    def on_enter(self):
        print("Entering Handle Crossing State")
        self.robot.display.show_message("Handle crossing", 0)

        # Stop the robot
        self.robot.display.show_message("Stop", 1)
        self.robot.enable_follow_the_line(False)
        self.robot.movement_controller.drive_desired_state(0, Direction.FORWARD)
        # sleep_ms(1000)

        # # Ask to drive until distance is reached
        # # self.robot.enable_speed_regulation(True)
        # self.robot.display.show_message("Go to cross", 1)
        # self.robot.movement_controller.register_distance_reached_alarm(self.DISTANCE_SENSORS_TO_CENTER)
        # self.robot.movement_controller.drive_desired_state(0.25, Direction.FORWARD)

    def on_exit(self):
        print("Exiting Handle Crossing State")

    def handle_event(self, event):
        if isinstance(event, TurningFinished):
            return FollowTheLine(self.robot)
        elif isinstance(event, DistanceReached):
            self.robot.movement_controller.drive_desired_state(0, Direction.FORWARD)
            self.robot.display.show_message("On the cross", 1)
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

# Define State Machine
class RobotStateMachine:
    def __init__(self, robot):
        self.robot = robot
        self.state = Idle(robot)
        self.state.on_enter()

    def handle_event(self, event):
        if isinstance(event, EmergencyEvent):
            new_state = EmergencyState(self.robot)
        else:
            new_state = self.state.handle_event(event)
        
        if new_state != self.state:
            self.state.on_exit()
            self.state = new_state
            self.state.on_enter()
