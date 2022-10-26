import rclpy
from abc import ABC, abstractmethod

import nxt_maze_solving.util.helper_classes as helper_classes

import nxt_msgs2.msg
import geometry_msgs.msg

from typing import List


class Robot(ABC, rclpy.node.Node):
    def __init__(self, name, maze_properties: helper_classes.MazeProperties):
        super().__init__(name)

        self.maze_propeties = maze_properties

        self._je_pub = self.create_publisher(
            nxt_msgs2.msg.JointEffort, "joint_effort", 10
        )

        self._cmd_vel_pub = self.create_publisher(
            geometry_msgs.msg.TwistStamped, "cmd_vel", 10
        )

        self.straight_forward_velocity: float = 0.0565
        self.end_scan_forward_velocity: float = 0.6772
        self.scanning_intersection: bool = False

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
    def stop_driving_motors(self):
        pass

    @abstractmethod
    def scan_intersection(self, color_values: List[helper_classes.Color]):
        pass

    @abstractmethod
    def realign(self, color_values: List[helper_classes.Color]):
        pass

    @abstractmethod
    def reset_realign(self):
        pass

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
