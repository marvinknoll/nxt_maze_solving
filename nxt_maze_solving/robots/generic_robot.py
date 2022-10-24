import rclpy
from abc import ABC, abstractmethod

import nxt_maze_solving.util.helper_classes as helper_classes

import nxt_msgs2
from typing import List


class Robot(ABC, rclpy.node.Node):
    def __init__(self, name, maze_properties: helper_classes.MazeProperties):
        super().__init__(name)

        self.maze_propeties = maze_properties

        self._je_pub = self.create_publisher(
            nxt_msgs2.msg.JointEffort, "joint_effort", 10
        )

        self.straight_forward_effort: float = 50.0
        self.end_scan_forward_effort: float = 20.0
        self.scanning_intersection: bool = False
        self.realigning: bool = False

        self._je_r = nxt_msgs2.msg.JointEffort()
        self._je_r.joint_name = "wheel_motor_r"
        self._je_l = nxt_msgs2.msg.JointEffort()
        self._je_l.joint_name = "wheel_motor_l"

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
    def stop_motors(self):
        pass

    @abstractmethod
    def scan_intersection(self, color_values: List[helper_classes.Color]):
        pass

    @abstractmethod
    def realign(self):
        pass

    @abstractmethod
    def reset_realign(self):
        pass

    def drive_straight(self, effort: float):
        # self.get_logger().info("driving straight")
        self._je_r.header.stamp = self.get_clock().now().to_msg()
        self._je_r.effort = -effort

        self._je_l.header.stamp = self.get_clock().now().to_msg()
        self._je_l.effort = -effort

        self._je_pub.publish(self._je_r)
        self._je_pub.publish(self._je_l)

    def stop_motors(self):
        # self.get_logger().info("Stopping motors")
        self._je_r.header.stamp = self.get_clock().now().to_msg()
        self._je_r.effort = 0.0

        self._je_l.header.stamp = self.get_clock().now().to_msg()
        self._je_l.effort = 0.0

        self._je_pub.publish(self._je_r)
        self._je_pub.publish(self._je_l)

    def turn_right(self, effort: float):
        # self.get_logger().info("turning right")
        self._je_r.header.stamp = self.get_clock().now().to_msg()
        self._je_r.effort = effort

        self._je_l.header.stamp = self.get_clock().now().to_msg()
        self._je_l.effort = -effort

        self._je_pub.publish(self._je_r)
        self._je_pub.publish(self._je_l)

    def turn_left(self, effort: float):
        # self.get_logger().info("turning left")
        self._je_r.header.stamp = self.get_clock().now().to_msg()
        self._je_r.effort = -effort

        self._je_l.header.stamp = self.get_clock().now().to_msg()
        self._je_l.effort = effort

        self._je_pub.publish(self._je_r)
        self._je_pub.publish(self._je_l)
