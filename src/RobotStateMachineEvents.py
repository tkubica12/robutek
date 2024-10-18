from MotorsController import Direction, TurnDirection
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

class DistanceReached(Event):
    """
    You can register alert so this event is triggered when set distance is reached.
    This can be used for example to stop the robot after reaching certain distance, 
    eg. to travel distance between line sensors and center of wheels.
    """
    pass

class CenterDetected(Event):
    """
    Event is triggered when the center line is detected.
    """
    pass

class LeftDetected(Event):
    """
    Event is triggered when the left line is detected.
    """
    pass

class RightDetected(Event):
    """
    Event is triggered when the right line is detected.
    """
    pass

class CenterLost(Event):
    """
    Event is triggered when the center line is lost.
    """
    pass

class LeftLost(Event):
    """
    Event is triggered when the left line is lost.
    """
    pass

class RightLost(Event):
    """
    Event is triggered when the right line is lost.
    """
    pass

class CrossDetected(Event):
    """
    Event is triggered when crossing of lines is detected.
    """
    pass

class Stopped(Event):
    """
    Event is triggered when robot stops.
    """
    pass

# Emergency events
class EmergencyEvent(Event):
    """
    Base class for all emergency events.
    """
    pass

class CollisionDetected(EmergencyEvent):
    pass

class AllLinesLost(EmergencyEvent):
    pass