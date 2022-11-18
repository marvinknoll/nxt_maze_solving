import rclpy.node
from abc import ABC, abstractmethod

import nxt_maze_solving.util.helper_classes as helper_classes

import nxt_msgs2.msg
import nxt_msgs2.srv
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg

from typing import List, Union
import copy
import time


class Robot(ABC, rclpy.node.Node):
    def __init__(self, name, maze_properties: helper_classes.MazeProperties):
        super().__init__(name)

        self.maze_propeties = maze_properties
        self.straight_forward_velocity: float = 0.0565

        self._scan_state = None
        self._realign_state = None

        self.intersection_directions: List[
            helper_classes.IntersectionDirection
        ] = []

        self._last_odom_msg: Union[nav_msgs.msg.Odometry, None] = None
        self._last_joint_state_msg: Union[
            sensor_msgs.msg.JointState, None
        ] = None

        self._je_pub = self.create_publisher(
            nxt_msgs2.msg.JointEffort, "joint_effort", 10
        )

        self._cmd_vel_pub = self.create_publisher(
            geometry_msgs.msg.TwistStamped, "cmd_vel", 10
        )

        self._robot_action_pub = self.create_publisher(
            nxt_msgs2.msg.RobotAction, "robot_action", 10
        )

        self.create_subscription(
            nav_msgs.msg.Odometry, "odom", self._cb_odom, 10
        )

        self.create_subscription(
            sensor_msgs.msg.JointState, "joint_states", self._cb_js, 10
        )

        # GetBatteryLevel client
        self._get_battery_level_client = self.create_client(
            nxt_msgs2.srv.GetBatteryLevel, "/brick/get_battery_level"
        )
        while not self._get_battery_level_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "brick/get_battery_level service not available,"
                " waiting again..."
            )

    @abstractmethod
    def on_end(self, color_values: List[helper_classes.Color]) -> bool:
        pass

    @abstractmethod
    def on_intersection(
        self, color_values: List[helper_classes.Color]
    ) -> bool:
        pass

    @abstractmethod
    def off_line(self, color_values: List[helper_classes.Color]) -> bool:
        pass

    @abstractmethod
    def on_line(self, color_values: List[helper_classes.Color]) -> bool:
        pass

    @abstractmethod
    def on_start(self, color_values: List[helper_classes.Color]) -> bool:
        pass

    @abstractmethod
    def scan_intersection(self, color_values: List[helper_classes.Color]):
        pass

    @abstractmethod
    def realign(self, color_values: List[helper_classes.Color]):
        pass

    def _cb_odom(self, msg: nav_msgs.msg.Odometry):
        self._last_odom_msg = msg

    def _cb_js(self, msg: sensor_msgs.msg.JointState):
        self._last_joint_state_msg = msg

    def scanning_intersection(self):
        return self._scan_state is not None

    def realigning(self):
        return self._realign_state is not None

    def drive_straight(self, linear_velocity: float):
        twist_msg = geometry_msgs.msg.TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = linear_velocity
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        self._cmd_vel_pub.publish(twist_msg)

    def driving_motors_still(self):
        last_velocities = self._last_joint_state_msg.velocity
        r_motor_idx = self._last_joint_state_msg.name.index("wheel_motor_r")
        l_motor_idx = self._last_joint_state_msg.name.index("wheel_motor_l")
        r_velocity = last_velocities[r_motor_idx]
        l_velocity = last_velocities[l_motor_idx]

        return r_velocity == 0 and l_velocity == 0

    def stop_driving_motors(self):
        twist_msg = geometry_msgs.msg.TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        self._cmd_vel_pub.publish(twist_msg)

    def _turn(self, angular_velocity: float):
        twist_msg = geometry_msgs.msg.TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = angular_velocity

        self._cmd_vel_pub.publish(twist_msg)

    def turn_right(self, angular_velocity: float):
        self._turn(angular_velocity * -1)

    def turn_left(self, angular_velocity: float):
        self._turn(angular_velocity)

    def append_intersection_turn(
        self, direction: helper_classes.IntersectionDirection
    ):
        self.intersection_directions.append(direction)
        directions_string = [
            direction.value for direction in self.intersection_directions
        ]
        self.get_logger().info(
            "Directions taken at the last intersections: %s"
            % directions_string
        )

    def get_total_path(self) -> List[helper_classes.IntersectionDirection]:
        return copy.deepcopy(self.intersection_directions)

    def get_shortest_path(self) -> List[helper_classes.IntersectionDirection]:
        directions = copy.deepcopy(self.intersection_directions)

        while helper_classes.IntersectionDirection.BACK in directions:
            b_idx = directions.index(helper_classes.IntersectionDirection.BACK)
            string_to_sub = directions[b_idx - 1 : b_idx + 2]
            substitution = ""
            if string_to_sub == [
                helper_classes.IntersectionDirection.LEFT,
                helper_classes.IntersectionDirection.BACK,
                helper_classes.IntersectionDirection.RIGHT,
            ]:
                substitution = helper_classes.IntersectionDirection.BACK
            elif string_to_sub == [
                helper_classes.IntersectionDirection.LEFT,
                helper_classes.IntersectionDirection.BACK,
                helper_classes.IntersectionDirection.STRAIGHT,
            ]:
                substitution = helper_classes.IntersectionDirection.RIGHT
            elif string_to_sub == [
                helper_classes.IntersectionDirection.RIGHT,
                helper_classes.IntersectionDirection.BACK,
                helper_classes.IntersectionDirection.LEFT,
            ]:
                substitution = helper_classes.IntersectionDirection.BACK
            elif string_to_sub == [
                helper_classes.IntersectionDirection.STRAIGHT,
                helper_classes.IntersectionDirection.BACK,
                helper_classes.IntersectionDirection.LEFT,
            ]:
                substitution = helper_classes.IntersectionDirection.RIGHT
            elif string_to_sub == [
                helper_classes.IntersectionDirection.STRAIGHT,
                helper_classes.IntersectionDirection.BACK,
                helper_classes.IntersectionDirection.STRAIGHT,
            ]:
                substitution = helper_classes.IntersectionDirection.BACK
            elif string_to_sub == [
                helper_classes.IntersectionDirection.LEFT,
                helper_classes.IntersectionDirection.BACK,
                helper_classes.IntersectionDirection.LEFT,
            ]:
                substitution = helper_classes.IntersectionDirection.STRAIGHT

            directions = (
                directions[: b_idx - 1]
                + [substitution]
                + directions[b_idx + 2 :]
            )

        return directions

    def send_start_benchmark_message(self, num_battery_requests: int = 20):
        measurements = []
        battery_level_req = nxt_msgs2.srv.GetBatteryLevel.Request()

        for i in range(num_battery_requests):
            future = self._get_battery_level_client.call_async(
                battery_level_req
            )
            future.add_done_callback(
                lambda future: measurements.append(
                    future.result().battery_voltage
                )
            )

            time.sleep(0.5)

            if i == num_battery_requests - 1:
                future.add_done_callback(
                    lambda future: self._send_benchmark_message(
                        int(sum(measurements) / len(measurements)), "start"
                    )
                )

    def send_end_benchmark_message(self, num_battery_requests: int = 20):
        measurements = []
        battery_level_req = nxt_msgs2.srv.GetBatteryLevel.Request()

        for i in range(num_battery_requests):
            future = self._get_battery_level_client.call_async(
                battery_level_req
            )
            future.add_done_callback(
                lambda future: measurements.append(
                    future.result().battery_voltage
                )
            )

            time.sleep(0.5)

            if i == num_battery_requests - 1:
                future.add_done_callback(
                    lambda future: self._send_benchmark_message(
                        int(sum(measurements) / len(measurements)), "end"
                    )
                )

    def send_start_realign_benchmark_message(self):
        battery_level_req = nxt_msgs2.srv.GetBatteryLevel.Request()
        future = self._get_battery_level_client.call_async(battery_level_req)
        future.add_done_callback(
            lambda future: self._send_benchmark_message(
                future.result().battery_voltage, "start_realign"
            )
        )

    def send_end_realign_benchmark_message(self):
        battery_level_req = nxt_msgs2.srv.GetBatteryLevel.Request()
        future = self._get_battery_level_client.call_async(battery_level_req)
        future.add_done_callback(
            lambda future: self._send_benchmark_message(
                future.result().battery_voltage, "end_realign"
            )
        )

    def send_start_intersection_scan_benchmark_message(self):
        battery_level_req = nxt_msgs2.srv.GetBatteryLevel.Request()
        future = self._get_battery_level_client.call_async(battery_level_req)
        future.add_done_callback(
            lambda future: self._send_benchmark_message(
                future.result().battery_voltage, "start_intersection_scan"
            )
        )

    def send_end_intersection_scan_benchmark_message(self):
        battery_level_req = nxt_msgs2.srv.GetBatteryLevel.Request()
        future = self._get_battery_level_client.call_async(battery_level_req)
        future.add_done_callback(
            lambda future: self._send_benchmark_message(
                future.result().battery_voltage, "end_intersection_scan"
            )
        )

    def _send_benchmark_message(self, battery_voltage, action):
        msg = nxt_msgs2.msg.RobotAction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.action = action
        msg.battery_voltage = battery_voltage
        self._robot_action_pub.publish(msg)
