import rclpy.action

import nxt_msgs2
import nav_msgs

import nxt_maze_solving.util.helper_functions as helper_functions
import nxt_maze_solving.util.helper_classes as helper_classes
import nxt_maze_solving.robots.generic_robot as generic_robot

import math

from typing import List, Union


class OneFixedSensorRobot(generic_robot.Robot):
    def __init__(self, name, maze_properties):
        super().__init__(name, maze_properties)

        self._realign_positions = [20, -40, 60, -60, -5]
        self._realign_effort = 20.0
        self._realign_start_yaw_z = None
        self._realign_idx = 0

        self._scan_reposition_effort = 40.0
        self._scan_turning_effort = 30.0
        self._scan_start_position = None
        self._scan_intersection_centre_odom = None
        self._scan_turned_left = False

        self._action_client = rclpy.action.ActionClient(
            self, nxt_msgs2.action.TurnMotor, "wheel_motor_r_turn"
        )

        self._last_odom_msg: Union[nav_msgs.msg.Odometry, None] = None
        self.create_subscription(
            nav_msgs.msg.Odometry, "odom", self._cb_odom, 10
        )

    def on_end(self, color_values: List[helper_classes.Color]) -> bool:
        return color_values[1] == self.maze_propeties.end_color

    def on_intersection(
        self, color_values: List[helper_classes.Color]
    ) -> bool:
        return color_values[1] == self.maze_propeties.intersection_color

    def off_line(self, color_values: List[helper_classes.Color]) -> bool:
        return color_values[1] == self.maze_propeties.background_color

    def on_line(self, color_values: List[helper_classes.Color]) -> bool:
        return color_values[1] == self.maze_propeties.line_color

    def on_start(self, color_values: List[helper_classes.Color]) -> bool:
        return color_values[1] == self.maze_propeties.start_color

    def scan_intersection(self, color_values: List[helper_classes.Color]):
        if self._scan_start_position is None:
            self._scan_start_position = self._last_odom_msg.pose.pose.position

        current_position = self._last_odom_msg.pose.pose.position
        delta_x = abs(current_position.x - self._scan_start_position.x)
        delta_y = abs(current_position.y - self._scan_start_position.y)
        dist = math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))

        if dist < 0.11:  # reposition on intersection centre
            self.drive_straight(self._scan_reposition_effort)
            return

        if self._scan_intersection_centre_odom is None:
            self._scan_intersection_centre_odom = self._last_odom_msg

        initial_yaw_z = helper_functions.orientation_deg_from_odom(
            self._scan_intersection_centre_odom
        )
        current_yaw_z = helper_functions.orientation_deg_from_odom(
            self._last_odom_msg
        )

        if not self._scan_turned_left:
            # Turn left to initial 360° scan position from various orientations
            if initial_yaw_z > 45 and initial_yaw_z < 135:
                # Faced WEST at start
                if (current_yaw_z > 45 and current_yaw_z < 180) or (
                    current_yaw_z < -135 and current_yaw_z > -180
                ):
                    self.turn_left(self._scan_turning_effort)
                else:
                    self._scan_turned_left = True
            elif initial_yaw_z < 45 and initial_yaw_z > -45:
                # Faced NORTH at start
                if current_yaw_z < 135:
                    self.turn_left(self._scan_turning_effort)
                else:
                    self._scan_turned_left = True
            elif initial_yaw_z < -45 and initial_yaw_z > -135:
                # Faced EAST at start
                if current_yaw_z < 45:
                    self.turn_left(self._scan_turning_effort)
                else:
                    self._scan_turned_left = True
            elif (initial_yaw_z < -135 and delta_y > -180) or (
                initial_yaw_z > 135 and initial_yaw_z < 180
            ):
                # Faced SOUTH at start
                if (current_yaw_z > 135 and current_yaw_z > 180) or (
                    current_yaw_z < -45 and current_yaw_z > -180
                ):
                    self.turn_left(self._scan_turning_effort)
                else:
                    self._scan_turned_left = True
        else:
            # Turn right and scan
            self.turn_right(self._scan_turning_effort)

        if not self._scan_turned_left:
            return

        if color_values[1] == self.maze_propeties.line_color:
            if current_yaw_z > 45 and current_yaw_z < 135:  # Facing WEST
                self.get_logger().info("Took WEST Path")
            elif current_yaw_z < 45 and current_yaw_z > -45:  # Facing NORTH
                self.get_logger().info("Took NORTH Path")
            elif current_yaw_z < -45 and current_yaw_z > -135:  # Facing EAST
                self.get_logger().info("Took EAST Path")
            elif (current_yaw_z < -135 and delta_y > -180) or (  # Facing SOUTH
                current_yaw_z > 135 and current_yaw_z < 180
            ):
                self.get_logger().info("Took SOUTH Path")

            self.reset_intersection_scan()

    def reset_intersection_scan(self):
        self.scanning_intersection = False
        self._scan_start_position = None
        self._scan_intersection_centre_odom = None
        self._scan_turned_left = False

    def realign(self):
        if self._last_odom_msg is None:
            return

        if self._realign_start_yaw_z is None:
            self.stop_motors()
            start_orientation = self._last_odom_msg.pose.pose.orientation
            _, _, _yaw_z = helper_functions.euler_from_quaternion(
                start_orientation.x,
                start_orientation.y,
                start_orientation.z,
                start_orientation.w,
            )
            self._realign_start_yaw_z = math.degrees(_yaw_z)
        else:
            current_orientation = self._last_odom_msg.pose.pose.orientation

            _, _, current_yaw_z = helper_functions.euler_from_quaternion(
                current_orientation.x,
                current_orientation.y,
                current_orientation.z,
                current_orientation.w,
            )

            delt_yaw_z = (
                math.degrees(current_yaw_z) - self._realign_start_yaw_z
            )

            current_turning_goal = self._realign_positions[self._realign_idx]

            if current_turning_goal > 0 and delt_yaw_z < current_turning_goal:
                self.turn_left(self._realign_effort)
            elif (
                current_turning_goal < 0 and delt_yaw_z > current_turning_goal
            ):
                self.turn_right(self._realign_effort)
            else:
                if self._realign_idx < len(self._realign_positions) - 1:
                    self.stop_motors()
                    self._realign_idx += 1
                else:
                    self.get_logger().info("Realignment failed")
                    self.stop_motors()

    def reset_realign(self):
        self._realign_idx = 0
        self._realign_start_yaw_z = None

    def _cb_odom(self, msg: nav_msgs.msg.Odometry):
        self._last_odom_msg = msg
