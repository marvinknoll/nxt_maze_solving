import nxt_msgs2.msg

import nxt_maze_solving.util.helper_functions as helper_functions
import nxt_maze_solving.util.helper_classes as helper_classes
import nxt_maze_solving.util.robot_states as robot_states
import nxt_maze_solving.robots.generic_robot as generic_robot

import math

from typing import List


class OneTurningSensorRobot(generic_robot.Robot):
    def __init__(self, maze_properties):
        super().__init__("one_turning_sensor", maze_properties)

        self._line_angle = None

        self._je_sensor_motor_pub = self.create_publisher(
            nxt_msgs2.msg.JointEffort, "joint_effort", 10
        )

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
                0.065,
                0.0456,
                TurnSensorToState(self, 105, 20.0, ScanState(self)),
            )

        self._scan_state = self._scan_state.on_event(color_values)

    def realign(self, color_values: List[helper_classes.Color]):
        if self._realign_state is None:
            self._realign_state = robot_states.RealignState(self, True)

        self._realign_state = self._realign_state.on_event(color_values)

    def _turn_sensor_motor(self, effort: float):
        msg = nxt_msgs2.msg.JointEffort()
        msg.joint_name = "sensor_motor"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.effort = effort
        self._je_sensor_motor_pub.publish(msg)


# Sensor configuartions specific states
class TurnSensorToState(robot_states.State):
    def __init__(
        self,
        robot: OneTurningSensorRobot,
        goal_pos: float,
        effort: float,
        next_state: robot_states.State,
    ):
        self._robot = robot
        self._goal_pos = goal_pos
        self._effort = effort
        self._next_state = next_state

        self._trashold_deg_left = 20
        self._trashold_deg_right = 20

        if self._goal_pos > 105 or self._goal_pos < -105:
            self._robot.get_logger().info(
                "ERROR: TurnSensorToState got invalid goal pos"
            )

    def on_event(self, color_values: List[helper_classes.Color]):
        if not self._robot.driving_motors_still():
            self._robot.stop_driving_motors()
            return self

        sensor_motor_idx = self._robot._last_joint_state_msg.name.index(
            "sensor"
        )
        sensor_position = math.degrees(
            self._robot._last_joint_state_msg.position[sensor_motor_idx]
        )

        if sensor_position > self._goal_pos + self._trashold_deg_left:
            self._robot._turn_sensor_motor(self._effort)
            return self
        elif sensor_position < self._goal_pos - self._trashold_deg_right:
            self._robot._turn_sensor_motor(-self._effort)
            return self
        else:
            self._robot._turn_sensor_motor(0.0)
            return self._next_state


class ScanState(robot_states.State):
    def __init__(self, robot: OneTurningSensorRobot, effort: float = 20.0):
        self._robot = robot
        self._effort = effort

    def on_event(self, color_values: List[helper_classes.Color]):
        sensor_idx = self._robot._last_joint_state_msg.name.index("sensor")
        sensor_pos = math.degrees(
            self._robot._last_joint_state_msg.position[sensor_idx]
        )

        if sensor_pos > -20:
            self._robot._turn_sensor_motor(self._effort)
            if (
                color_values[1] == self._robot.maze_propeties.line_color
                and self._robot._line_angle is None
            ):
                self._robot._line_angle = sensor_pos
            return self
        else:
            return TurnSensorToState(
                self._robot,
                0.0,
                20.0,
                RepositionRobotCentreState(self._robot, 0.05, 0.0456),
            )


class RepositionRobotCentreState(robot_states.State):
    def __init__(
        self,
        robot: OneTurningSensorRobot,
        goal_linear_distance: float,
        linear_velocity: float,
    ):
        self._robot = robot
        self._goal_linear_distance = goal_linear_distance
        self._linear_velocity = linear_velocity
        self._initial_pose = None
        if self._robot._last_odom_msg is not None:
            self._initial_pose = self._robot._last_odom_msg.pose.pose.position

    def on_event(self, color_values: List[helper_classes.Color]):
        if self._initial_pose is None:
            if self._robot._last_odom_msg is not None:
                self._initial_pose = (
                    self._robot._last_odom_msg.pose.pose.position
                )
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
            self._robot.stop_driving_motors()
            return ReorientRobotState(self._robot)


class ReorientRobotState(robot_states.State):
    def __init__(self, robot: OneTurningSensorRobot):
        self._robot = robot
        self._initial_odom = self._robot._last_odom_msg
        self._angular_velocity = 0.4

    def on_event(self, color_values: List[helper_classes.Color]):
        if self._robot._line_angle is not None:
            if self._robot._line_angle < 45 and self._robot._line_angle > -45:
                self._robot.append_intersection_turn(
                    helper_classes.IntersectionDirection.STRAIGHT
                )
                return ScanIntersectionEndState(self._robot)

            if self._robot._line_angle > 45:
                deg_turned_left = helper_functions.degrees_turned_from(
                    self._initial_odom,
                    self._robot._last_odom_msg,
                    helper_classes.TurningDirection.LEFT,
                )
                if (
                    color_values[1] != self._robot.maze_propeties.line_color
                    or deg_turned_left < 45
                    or deg_turned_left > 135
                ):
                    self._robot.turn_left(self._angular_velocity)
                    return self
                else:
                    self._robot.append_intersection_turn(
                        helper_classes.IntersectionDirection.LEFT
                    )
                    self._robot.stop_driving_motors()
                    return ScanIntersectionEndState(self._robot)

        if color_values[1] != self._robot.maze_propeties.line_color:
            self._robot.turn_right(self._angular_velocity)
            return self

        self._robot.stop_driving_motors()

        current_odom = self._robot._last_odom_msg
        deg_turned_right = helper_functions.degrees_turned_from(
            self._initial_odom,
            current_odom,
            helper_classes.TurningDirection.RIGHT,
        )

        if deg_turned_right > 45 and deg_turned_right < 135:
            self._robot.append_intersection_turn(
                helper_classes.IntersectionDirection.RIGHT
            )
            return ScanIntersectionEndState(self._robot)
        elif deg_turned_right > 135 and deg_turned_right < 225:
            self._robot.append_intersection_turn(
                helper_classes.IntersectionDirection.BACK
            )
            return ScanIntersectionEndState(self._robot)
        elif deg_turned_right > 225:
            self._robot.get_logger().info(
                "ERROR while reorienting on intersection"
            )
            return self


class ScanIntersectionEndState(robot_states.State):
    def __init__(self, robot: OneTurningSensorRobot):
        self._robot = robot

    def on_event(self, color_values: List[helper_classes.Color]):
        self._robot._line_angle = None
        return None
