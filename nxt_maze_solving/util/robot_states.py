import nxt_maze_solving.util.helper_classes as helper_classes
import nxt_maze_solving.util.helper_functions as helper_functions

import nxt_maze_solving.robots.generic_robot as generic_robot

import math

from typing import List


class State:
    def __init__(self):
        print("Initialized following state:", str(self))

    def on_event(self, color_values: List[helper_classes.Color]):
        pass

    def __repr__(self):

        return self.__str__()

    def __str__(self):
        return self.__class__.__name__


class RealignState(State):
    def __init__(
        self,
        robot: generic_robot.Robot,
        realign_base_velocity: float = 0.29,
    ):
        self._robot = robot
        self._realign_degrees = [20, -40, 80, -120, 60]
        self._realign_idx = 0
        self._realign_base_velocity = realign_base_velocity

        self._robot.stop_driving_motors()

        self._initial_odom = None
        if self._robot._last_odom_msg is not None:
            self._initial_odom = self._robot._last_odom_msg

    def on_event(self, color_values: List[helper_classes.Color]):
        if color_values[1] != self._robot.maze_propeties.background_color:
            self._robot.stop_driving_motors()
            if self._robot.driving_motors_still():
                return None
            else:
                # Wait for motors to be still
                return self

        if self._initial_odom is None:
            if self._robot._last_odom_msg is not None:
                self._initial_odom = self._robot._last_odom_msg
            else:
                return self

        current_odom = self._robot._last_odom_msg
        current_turning_goal_deg = self._realign_degrees[self._realign_idx]

        realign_velocity = self._realign_base_velocity
        if self._realign_idx < 2:
            realign_velocity = self._realign_base_velocity * 0.75

        if color_values[1] == self._robot.maze_propeties.background_color:
            if current_turning_goal_deg > 0:
                deg_turned_left = helper_functions.degrees_turned_from(
                    self._initial_odom,
                    current_odom,
                    helper_classes.TurningDirection.LEFT,
                )
                if (
                    deg_turned_left < abs(current_turning_goal_deg)
                    or deg_turned_left > 275
                ):
                    self._robot.turn_left(realign_velocity)
                    return self
            else:
                deg_turned_right = helper_functions.degrees_turned_from(
                    self._initial_odom,
                    current_odom,
                    helper_classes.TurningDirection.RIGHT,
                )
                if (
                    deg_turned_right < abs(current_turning_goal_deg)
                    or deg_turned_right > 275
                ):
                    self._robot.turn_right(realign_velocity)
                    return self

            if self._realign_idx < len(self._realign_degrees) - 1:
                self._initial_odom = current_odom
                self._realign_idx += 1
                return self
            else:
                self._robot.get_logger().info("Failed to realign")
                return None


class RepositionForScanState(State):
    def __init__(
        self,
        robot: generic_robot.Robot,
        goal_linear_distance: float,
        linear_velocity: float,
        next_state: State,
    ):
        self._robot = robot
        self._initial_pose = None
        self._goal_linear_distance = goal_linear_distance
        self._linear_velocity = linear_velocity
        self._next_state = next_state
        if self._robot._last_odom_msg is not None:
            self._initial_pose = self._robot._last_odom_msg.pose.pose.position

    def on_event(self, color_values: List[helper_classes.Color]):
        if self._initial_pose is None:
            if self._robot._last_odom_msg is not None:
                self._initial_pose = (
                    self._robot._last_odom_msg.pose.pose.position
                )
                return self
            else:
                return self

        current_position = self._robot._last_odom_msg.pose.pose.position
        delta_x = abs(current_position.x - self._initial_pose.x)
        delta_y = abs(current_position.y - self._initial_pose.y)
        current_distance = math.sqrt(delta_x**2 + delta_y**2)

        if current_distance < self._goal_linear_distance:
            self._robot.drive_straight(self._linear_velocity)
            return self
        else:
            return self._next_state


class TurnRobotState(State):
    def __init__(
        self,
        robot: generic_robot.Robot,
        next_state: State,
        turning_deg: float,
        turning_direction: helper_classes.TurningDirection,
        turning_velocity: float = 0.5079,
    ):
        self._robot = robot
        self._next_state = next_state
        self._turning_deg = turning_deg
        self._turning_direction = turning_direction
        self._turning_velocity = turning_velocity
        self._initial_odom = None
        if self._robot._last_odom_msg is not None:
            self._initial_odom = self._robot._last_odom_msg

    def on_event(self, color_values: List[helper_classes.Color]):
        if self._initial_odom is None:
            if self._robot._last_odom_msg is not None:
                self._initial_odom = self._robot._last_odom_msg
            else:
                return self

        current_odom = self._robot._last_odom_msg
        deg_turned = helper_functions.degrees_turned_from(
            self._initial_odom,
            current_odom,
            self._turning_direction,
        )
        if (
            deg_turned < self._turning_deg or deg_turned > 350
        ):  # or deg_turned > 450 to compensate odom inaccuracy
            if self._turning_direction == helper_classes.TurningDirection.LEFT:
                self._robot.turn_left(self._turning_velocity)
            else:
                self._robot.turn_right(self._turning_velocity)
            return self
        else:
            return self._next_state
