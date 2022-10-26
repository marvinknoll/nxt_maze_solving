import rclpy
import rclpy.executors
import rclpy.node
import rclpy.action
import rclpy.time
import rclpy.duration
import rclpy.clock
import rclpy.callback_groups

import nxt_msgs2.msg
import nxt_msgs2.action
import nxt_msgs2.srv

import nxt_maze_solving.util.helper_functions as helper_functions
import nxt_maze_solving.util.helper_classes as helper_classes
import nxt_maze_solving.robots.generic_robot as Robot
import nxt_maze_solving.robots.one_fixed_sensor_robot as one_fixed_sensor_robot


from typing import List


class MazeSolver(rclpy.node.Node):
    def __init__(
        self,
        name: str,
        robot: Robot,
    ):
        super().__init__(name)
        self._robot = robot

        self._end_color_measured_cnt = 0

        self.colors: List[helper_classes.Color] = [
            helper_classes.Color.GREEN,
            helper_classes.Color.GREEN,
            helper_classes.Color.GREEN,
        ]  # [sensor_l, sensor_c, sensor_r]

        self.create_subscription(
            nxt_msgs2.msg.Color, "color_sensor", self._cb_c_color_read, 10
        )

        self._je_pub = self.create_publisher(
            nxt_msgs2.msg.JointEffort, "joint_effort", 10
        )

        self.create_timer(0.1, self.solve_maze)

    def _cb_c_color_read(self, color_msg: nxt_msgs2.msg.Color):
        self.colors[1] = helper_functions.color_rgba_to_color(color_msg)

    def solve_maze(self):
        if (
            self._robot.on_intersection(self.colors)
            or self._robot.scanning_intersection
        ):
            if not self._robot.scanning_intersection:
                # self.get_logger().info("detected intersection")
                self._robot.scanning_intersection = True

            self._robot.scan_intersection(self.colors)
            return

        if self._robot.on_end(self.colors):
            # self.get_logger().info("detected end_color")
            self._end_color_measured_cnt += 1
            if self._end_color_measured_cnt > 6:
                self._robot.stop_driving_motors()

        if self._robot.off_line(self.colors):
            # self.get_logger().info("off_line")
            self._robot.realign(self.colors)

        if self._robot.on_line(self.colors):
            # self.get_logger().info("on_line")
            self._robot.drive_straight(self._robot.straight_forward_velocity)
            self._robot.reset_realign()

        if self._robot.on_start(self.colors):
            # self.get_logger().info("on_start")
            self._robot.drive_straight(self._robot.straight_forward_velocity)

        if not self._robot.on_end(self.colors):
            # self.get_logger().info("resetting _end_color_measured_cnt")
            self._end_color_measured_cnt = 0

    def destroy_node(self):
        """Destroy the node."""
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        executor = rclpy.executors.MultiThreadedExecutor()

        maze_properties = helper_classes.MazeProperties(
            line_color=helper_classes.Color.BLACK,
            background_color=helper_classes.Color.WHITE,
            start_color=helper_classes.Color.GREEN,
            intersection_color=helper_classes.Color.RED,
            end_color=helper_classes.Color.BLUE,
        )
        one_fix_sensor_robot_node = one_fixed_sensor_robot.OneFixedSensorRobot(
            "one_fix_sensor", maze_properties
        )
        maze_solver = MazeSolver("maze_runner", one_fix_sensor_robot_node)

        try:

            executor.add_node(maze_solver)
            executor.add_node(one_fix_sensor_robot_node)
            executor.spin()
        finally:
            maze_solver.destroy_node()

            executor.shutdown()

    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
