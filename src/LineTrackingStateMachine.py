from RobotStateMachineStates import OnCross, DistanceReached
from MotorsController import Direction
from time import sleep_ms
from machine import Timer

# Define Events
class LineTrackingEvent:
    """
    Base class for all line tracking events.
    """
    pass

class CenterDetected(LineTrackingEvent):
    """
    Event is triggered when the center line is detected.
    """
    pass

class LeftDetected(LineTrackingEvent):
    """
    Event is triggered when the left line is detected.
    """
    pass

class RightDetected(LineTrackingEvent):
    """
    Event is triggered when the right line is detected.
    """
    pass

class CenterLost(LineTrackingEvent):
    """
    Event is triggered when the center line is lost.
    """
    pass

class LeftLost(LineTrackingEvent):
    """
    Event is triggered when the left line is lost.
    """
    pass

class RightLost(LineTrackingEvent):
    """
    Event is triggered when the right line is lost.
    """
    pass

class CrossDetected(LineTrackingEvent):
    """
    Event is triggered when crossing of lines is detected.
    """
    pass

class Stopped(LineTrackingEvent):
    """
    Event is triggered when robot stops.
    """
    pass

# Define States
class LineTrackingState:
    def __init__(self, line_tracking):
        self.line_tracking = line_tracking

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def handle_event(self, event):
        pass

class FollowTheLineInactive(LineTrackingState):
    def on_enter(self):
        print("Entering FollowTheLineInactive State")

    def on_exit(self):
        print("Exiting FollowTheLineInactive State")

    def handle_event(self, event):
        return self

class InCenter(LineTrackingState):
    _instance = None

    def __new__(cls, line_tracking):
        if cls._instance is None:
            cls._instance = super(InCenter, cls).__new__(cls)
            cls._instance.line_tracking = line_tracking
        return cls._instance
    
    def on_enter(self):
        print("Entering InCenter State")
        self.line_tracking.display.show_message("In Center", 2)
        self.line_tracking.set_angular_velocity(0)
        self.line_tracking.stop_regulation()
        self.line_tracking.follow_the_line_drive()

    def on_exit(self):
        print("Exiting InCenter State")
        self.line_tracking.display.show_message("", 2)

    def handle_event(self, event):
        if isinstance(event, LeftDetected):
            return CorrectionLeft(self.line_tracking)
        elif isinstance(event, RightDetected):
            return CorrectionRight(self.line_tracking)
        elif isinstance(event, CrossDetected):
            return CrossStopping(self.line_tracking)
        return self

class CorrectionLeft(LineTrackingState):
    def on_enter(self):
        print("Entering CorrectionLeft State")
        self.line_tracking.display.show_message("Correction Left", 2)
        self.line_tracking.set_angular_velocity(-1.0)
        self.line_tracking.start_regulation()
        self.line_tracking.correction_action()

    def on_exit(self):
        print("Exiting CorrectionLeft State")
        self.line_tracking.display.show_message("", 2)

    def handle_event(self, event):
        if isinstance(event, CenterDetected):
            return InCenter(self.line_tracking)
        elif isinstance(event, RightDetected):
            return CorrectionRight(self.line_tracking)
        elif isinstance(event, CrossDetected):
            return CrossStopping(self.line_tracking)
        return self

class CorrectionRight(LineTrackingState):
    def on_enter(self):
        print("Entering CorrectionRight State")
        self.line_tracking.display.show_message("Correction Right", 2)
        self.line_tracking.set_angular_velocity(1.0)
        self.line_tracking.start_regulation()
        self.line_tracking.correction_action()

    def on_exit(self):
        print("Exiting CorrectionRight State")
        self.line_tracking.display.show_message("", 2)

    def handle_event(self, event):
        if isinstance(event, CenterDetected):
            return InCenter(self.line_tracking)
        elif isinstance(event, LeftDetected):
            return CorrectionLeft(self.line_tracking)
        elif isinstance(event, CrossDetected):
            return CrossStopping(self.line_tracking)
        return self
    
class CrossCentering(LineTrackingState):
    def on_enter(self):
        print("Entering CrossCentering State")
        self.line_tracking.center_to_cross()
        # if self.line_tracking.is_on_cross():
        #     self.line_tracking.display.show_message("Cross Centering", 2)
        #     self.line_tracking.center_to_cross()
        # else:
        #     self.line_tracking.display.show_message("Cross overshoot", 2)
        #     self.line_tracking.drive_back_to_cross()

    def on_exit(self):
        print("Exiting CrossCentering State")
        self.line_tracking.display.show_message("", 2)

    def handle_event(self, event):
        if isinstance(event, CrossDetected):
            self.on_enter()
        if isinstance(event, DistanceReached):
            self.line_tracking.movement_controller.drive_desired_state(0, Direction.FORWARD)
            self.line_tracking.robot.robot_state_machine.handle_event(OnCross())
            return FollowTheLineInactive(self.line_tracking)
        return self

class CrossStopping(LineTrackingState):
    def on_enter(self):
        print("Entering CrossStopping State")
        self.line_tracking.stop()
        self.line_tracking.display.show_message("Cross Stopping", 2)
        self.line_tracking.set_angular_velocity(0)
        self.line_tracking.stop_regulation()
        self.line_tracking.robot.enable_follow_the_line(False)

        # Set up a timer to handle the delay non-blocking
        self.timer = Timer(3)
        self.timer.init(period=200, mode=Timer.ONE_SHOT, callback=self.on_timer_complete)

    def on_timer_complete(self, t):
        self.line_tracking.line_tracking_state_machine.handle_event(Stopped())

    def on_exit(self):
        print("Exiting CrossStopping State")
        self.line_tracking.display.show_message("", 2)
        # Deinitialize the timer if it exists
        if hasattr(self, 'timer'):
            self.timer.deinit()

    def handle_event(self, event):
        if isinstance(event, Stopped):
            return CrossCentering(self.line_tracking)
        return self
    
# Define State Machine
class LineTrackingStateMachine:
    def __init__(self, line_tracking):
        self.robot = line_tracking
        self.state = InCenter(line_tracking)
        self.state.on_enter()

    def handle_event(self, event):
        new_state = self.state.handle_event(event)
        if new_state != self.state:
            self.state.on_exit()
            self.state = new_state
            self.state.on_enter()