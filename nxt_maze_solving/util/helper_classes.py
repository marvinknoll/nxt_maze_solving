import nxt_maze_solving.util.helper_classes as helper_classes
import enum
from typing import List


@enum.unique
class Color(enum.IntEnum):
    BLACK = 1
    BLUE = 2
    GREEN = 3
    YELLOW = 4
    RED = 5
    WHITE = 6


class MazeProperties:
    def __init__(
        self,
        line_color: helper_classes.Color,
        background_color: helper_classes.Color,
        start_color: helper_classes.Color,
        intersection_color: helper_classes.Color,
        end_color: helper_classes.Color,
    ):
        self.line_color = line_color
        self.background_color = background_color
        self.start_color = start_color
        self.intersection_color = intersection_color
        self.end_color = end_color


@enum.unique
class TurningDirection(enum.IntEnum):
    LEFT = 1
    RIGHT = 2
