import nxt_maze_solving.util.helper_functions as helper_functions
import nxt_maze_solving.util.helper_classes as helper_classes
import nxt_maze_solving.util.robot_states as robot_states
import nxt_maze_solving.robots.generic_robot as generic_robot

import math

from typing import List, Union


class OneFixedSensorRobot(generic_robot.Robot):
    def __init__(self, name, maze_properties):
        super().__init__(name, maze_properties)

        self._realign_positions = [20, -40, 60, -60, -5]
        self._realign_velocity = 0.29
        self._realign_start_yaw_z = None
        self._realign_idx = 0

        self._scan_reposition_velocity = 0.0456
        self._scan_turning_velocity = 0.5079
        self._scan_start_position = None
        self._scan_intersection_centre_odom = None
        self._scan_turned_left = False

    def on_end(self, color_values: List[helper_classes.Color]) -> bool:
        return color_values[1] == self.maze_propeties.end_color

    def on_intersection(
        self, color_values: List[helper_classes.Color]
    ) -> bool:
        return color_values[1] == self.maze_propeties.intersection_color

    def off_line(self, color_values: List[helper_classes.Color]) -> bool:
        return (
            color_values[1] == self.maze_propeties.background_color
            and not self.scanning_intersection()
        )

    def on_line(self, color_values: List[helper_classes.Color]) -> bool:
        if (
            self.maze_propeties.line_color == helper_classes.Color.BLACK
            and self.maze_propeties.background_color
            == helper_classes.Color.WHITE
        ):
            # If the sensor is at the border of the black line it sometimes
            # reads BLUE instead of BLACK
            return (
                color_values[1] == self.maze_propeties.line_color
                or color_values[1] == helper_classes.Color.BLUE
            )
        else:
            return color_values[1] == self.maze_propeties.line_color

    def on_start(self, color_values: List[helper_classes.Color]) -> bool:
        return color_values[1] == self.maze_propeties.start_color

    def scan_intersection(self, color_values: List[helper_classes.Color]):
        if self._scan_state is None:

            self._scan_state = robot_states.RepositionForScanState(
                self,
                0.1,
                0.0456,
                robot_states.TurnRobotState(
                    self,
                    ScanToRight(self),
                    135,
                    helper_classes.TurningDirection.LEFT,
                ),
            )

        self._scan_state = self._scan_state.on_event(color_values)

    def realign(self, color_values: List[helper_classes.Color]):
        if self._realign_state is None:
            self._realign_state = robot_states.RealignState(self)

        self._realign_state = self._realign_state.on_event(color_values)


# Sensor configuration specific states
class ScanToRight(robot_states.State):
    def __init__(self, robot: OneFixedSensorRobot, turn_velocity: float = 0.4):
        self._robot = robot
        self._turn_velocity = turn_velocity
        self._initial_odom = None
        self._line_found = False

    def on_event(self, color_values: List[helper_classes.Color]):
        if self._initial_odom is None:
            if self._robot._last_odom_msg is not None:
                self._initial_odom = self._robot._last_odom_msg
            else:
                return self

        if (
            color_values[1] != self._robot.maze_propeties.line_color
            and not self._line_found
        ):
            self._robot.turn_right(self._turn_velocity)
            return self
        else:
            self._robot.stop_driving_motors()

            if not self._line_found:
                self._line_found = True

                current_odom = self._robot._last_odom_msg
                deg_turned_right = helper_functions.degrees_turned_from(
                    self._initial_odom,
                    current_odom,
                    helper_classes.TurningDirection.RIGHT,
                )

                if deg_turned_right < 90:
                    self._robot.append_intersection_turn(
                        helper_classes.IntersectionDirection.LEFT
                    )
                elif deg_turned_right > 90 and deg_turned_right < 180:
                    self._robot.append_intersection_turn(
                        helper_classes.IntersectionDirection.STRAIGHT
                    )
                elif deg_turned_right > 180 and deg_turned_right < 270:
                    self._robot.append_intersection_turn(
                        helper_classes.IntersectionDirection.RIGHT
                    )
                else:
                    self._robot.append_intersection_turn(
                        helper_classes.IntersectionDirection.BACK
                    )

            if not self._robot.driving_motors_still():
                return self
            else:
                return None
