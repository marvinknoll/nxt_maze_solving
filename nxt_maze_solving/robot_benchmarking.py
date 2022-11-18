import rclpy
import rclpy.executors
import rclpy.node
import rclpy.action
import rclpy.time
import rclpy.duration
import rclpy.clock
import rclpy.callback_groups

import nxt_msgs2.msg
import nxt_msgs2.srv

import nxt_maze_solving.util.helper_functions as helper_functions

from datetime import datetime
import sys

from typing import List
import enum


@enum.unique
class RobotActionType(enum.IntEnum):
    REALIGN = 1
    INTERSECTION_SCAN = 2


class RobotAction:
    def __init__(
        self,
        action_type: RobotActionType,
        start_time_stamp,
        start_voltage: int,
    ):
        self.action_type = action_type

        self.start_time_stamp: rclpy.time.Time = start_time_stamp
        self.start_voltage: int = start_voltage

        self.end_time_stamp: rclpy.time.Time = None
        self.end_voltage: int = None

    def __repr__(self):
        s_dt = datetime.fromtimestamp(
            self.start_time_stamp.nanoseconds // 1000000000
        )
        e_dt = datetime.fromtimestamp(
            self.end_time_stamp.nanoseconds // 1000000000
        )
        duration = rclpy.time.Duration(
            nanoseconds=int(
                (self.end_time_stamp - self.start_time_stamp).nanoseconds
            )
        )
        duration_seconds = duration.nanoseconds / 1000000000

        return (
            f"(start_time: {s_dt.strftime('%Y-%m-%d %H:%M:%S')}, end_time:"
            f" {e_dt.strftime('%Y-%m-%d %H:%M:%S')}, duration:"
            f" {duration_seconds}s, start_voltage: {self.start_voltage},"
            f" end_voltage: {self.end_voltage})"
        )


class RealignmentAction(RobotAction):
    def __init__(self, start_time_stamp, start_voltage: int):
        super().__init__(
            RobotActionType.REALIGN, start_time_stamp, start_voltage
        )

    def __repr__(self):
        return "RealignmentAction" + super().__repr__()


class IntersectionScanAction(RobotAction):
    def __init__(self, start_time_stamp, start_voltage: int):
        super().__init__(
            RobotActionType.INTERSECTION_SCAN, start_time_stamp, start_voltage
        )

    def __repr__(self):
        return "IntersectionScanAction" + super().__repr__()


class RobotBenchmarking(rclpy.node.Node):
    def __init__(
        self,
    ):
        super().__init__("robot_benchmarking")

        self._start_time_stamp: rclpy.time.Time = None
        self._end_time_stamp: rclpy.time.Time = None

        self._start_voltage: int = None
        self._end_voltage: int = None

        self._realignments: List[RealignmentAction] = []
        self._intersection_scans: List[IntersectionScanAction] = []

        self.create_subscription(
            nxt_msgs2.msg.RobotAction,
            "/robot_action",
            self._cb_robot_action,
            10,
        )

    def _cb_robot_action(self, msg: nxt_msgs2.msg.RobotAction):
        action = msg.action
        stamp = rclpy.time.Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
            clock_type=rclpy.clock.ClockType.ROS_TIME,
        )
        if action == "start":
            # self.get_logger().info("GOT START BENCHMARK MSG")
            self._start_time_stamp = stamp
            self._start_voltage = msg.battery_voltage
        elif action == "end":
            # self.get_logger().info("GOT END BENCHMARK MSG")
            self._end_time_stamp = stamp
            self._end_voltage = msg.battery_voltage
            self.save_benchmarks_to_file()
        elif action == "start_realign":
            # self.get_logger().info("GOT START REALIGN BENCHMARK MSG")
            realign_action = RealignmentAction(stamp, msg.battery_voltage)
            self._realignments.append(realign_action)
        elif action == "end_realign":
            # self.get_logger().info("GOT END REALIGN BENCHMARK MSG")
            self._realignments[-1].end_time_stamp = stamp
            self._realignments[-1].end_voltage = msg.battery_voltage
        elif action == "start_intersection_scan":
            # self.get_logger().info("GOT START INTERSECTION BENCHMARK MSG")
            scan_action = IntersectionScanAction(stamp, msg.battery_voltage)
            self._intersection_scans.append(scan_action)
        elif action == "end_intersection_scan":
            # self.get_logger().info("GOT END INTERSECTION BENCHMARK MSG")
            self._intersection_scans[-1].end_time_stamp = stamp
            self._intersection_scans[-1].end_voltage = msg.battery_voltage

    def _get_total_duration(self) -> rclpy.time.Duration:
        return rclpy.time.Duration(
            nanoseconds=int(
                (self._end_time_stamp - self._start_time_stamp).nanoseconds
            )
        )

    def _calculate_actions_data(self, actions: List[RobotAction]):
        total_actions_duration = rclpy.time.Duration()
        actions_count = len(actions)

        for action in actions:
            action_duration = rclpy.time.Duration(
                nanoseconds=int(
                    (
                        action.end_time_stamp - action.start_time_stamp
                    ).nanoseconds
                )
            )
            total_actions_duration = helper_functions.add_durations(
                total_actions_duration, action_duration
            )

        return (
            total_actions_duration,
            actions_count,
        )

    def _data_complete(self) -> bool:
        data_complete = True
        recieved_end_time_stamp = self._end_time_stamp is not None
        data_complete = data_complete and recieved_end_time_stamp

        if len(self._realignments) > 0:
            realignments_complete = (
                self._realignments[-1].end_time_stamp is not None
            )
            data_complete = data_complete and realignments_complete

        if len(self._intersection_scans) > 0:
            intersection_scans_complete = (
                self._intersection_scans[-1].end_time_stamp is not None
            )
            data_complete = data_complete and intersection_scans_complete

        return data_complete

    def save_benchmarks_to_file(self):
        robot_configuration = sys.argv[1]
        start_time_str = datetime.fromtimestamp(
            +self._start_time_stamp.nanoseconds // 1000000000
        )
        end_time_str = datetime.fromtimestamp(
            +self._end_time_stamp.nanoseconds // 1000000000
        )
        (
            total_intersections_duration,
            intersections_count,
        ) = self._calculate_actions_data(self._intersection_scans)
        (
            total_realignments_duration,
            realignmets_count,
        ) = self._calculate_actions_data(self._realignments)

        file_name = robot_configuration + " " + str(datetime.now())
        with open("./benchmarks/%s" % file_name, "w") as file:
            file.write("Robot configuration: %s\n\n" % robot_configuration)
            file.write(
                "Total run:\n\tstart_time: %s\n\tend_time:"
                " %s\n\ttotal_duration: %ss\n\tstart_voltage:"
                " %s\n\tend_voltage: %s\n\n"
                % (
                    start_time_str,
                    end_time_str,
                    self._get_total_duration().nanoseconds / 1000000000,
                    self._start_voltage,
                    self._end_voltage,
                )
            )
            file.write(
                "Intersections:\n\ttotal_duration: %s\n\tcount: %s\n"
                % (
                    total_intersections_duration,
                    intersections_count,
                )
            )
            file.write("\tindividual intersection scans:\n")
            for intersection_scan in self._intersection_scans:
                file.write("\t\t%s\n" % intersection_scan)
            file.write("\n")

            file.write(
                "Realignments:\n\ttotal_duration: %s\n\tcount: %s\n"
                % (
                    total_realignments_duration,
                    realignmets_count,
                )
            )
            file.write("\tindividual realignments:\n")
            for realignment in self._realignments:
                file.write("\t\t%s\n" % realignment)


def main(args=None):
    try:
        rclpy.init(args=args)
        robot_benchmarking_node = RobotBenchmarking()
        rclpy.spin(robot_benchmarking_node)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Got clean shutdown signal, shutting down node.")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
