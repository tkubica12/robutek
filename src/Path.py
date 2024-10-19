class PathAction:
    """
    A PathAction is a single action to be taken on path
    """
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    GO_STRAIGHT = "GO_STRAIGHT"

class Path:
    """
    A Path is a list of PathElements and provides methods to manipulate and read path such as next action to take.
    """
    def __init__(self, path_actions: list[PathAction] = []):
        self.path_actions = path_actions
        self.current_index = 0

    def add_path_action(self, path_action: PathAction):
        self.path_actions.append(path_action)

    def next_action(self) -> PathAction:
        if self.current_index < len(self.path_actions):
            action = self.path_actions[self.current_index]
            self.current_index += 1
            return action
        return None