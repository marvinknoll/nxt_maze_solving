import nxt_msgs2.msg
import nav_msgs.msg

import nxt_maze_solving.util.helper_classes as helper_classes

import math


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def orientation_deg_from_odom(odom: nav_msgs.msg.Odometry):
    _, _, yaw_z = euler_from_quaternion(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,
    )
    return math.degrees(yaw_z)


def color_rgba_to_color(
    color_msg: nxt_msgs2.msg.Color,
) -> helper_classes.Color:
    if (
        color_msg.color.r == 0.0
        and color_msg.color.g == 0.0
        and color_msg.color.b == 0.0
    ):
        return helper_classes.Color.BLACK
    elif (
        color_msg.color.r == 0.0
        and color_msg.color.g == 0.0
        and color_msg.color.b == 255.0
    ):
        return helper_classes.Color.BLUE
    elif (
        color_msg.color.r == 0.0
        and color_msg.color.g == 255.0
        and color_msg.color.b == 0.0
    ):
        return helper_classes.Color.GREEN
    elif (
        color_msg.color.r == 255.0
        and color_msg.color.g == 255.0
        and color_msg.color.b == 0.0
    ):
        return helper_classes.Color.YELLOW
    elif (
        color_msg.color.r == 255.0
        and color_msg.color.g == 0.0
        and color_msg.color.b == 0.0
    ):
        return helper_classes.Color.RED
    elif (
        color_msg.color.r == 255.0
        and color_msg.color.g == 255.0
        and color_msg.color.b == 255.0
    ):
        return helper_classes.Color.WHITE
    else:
        raise Exception("Invalid color message")


def degrees_turned_from(
    initial_odom: nav_msgs.msg.Odometry,
    current_odom: nav_msgs.msg.Odometry,
    turning_left: bool,
):
    initial_yaw_z = orientation_deg_from_odom(initial_odom)
    current_yaw_z = orientation_deg_from_odom(current_odom)

    initial_yaw_z_norm = (
        initial_yaw_z if initial_yaw_z > 0 else 360 + initial_yaw_z
    )
    current_yaw_z_norm = (
        current_yaw_z if current_yaw_z > 0 else 360 + current_yaw_z
    )

    if turning_left:
        if initial_yaw_z_norm <= current_yaw_z_norm:
            return current_yaw_z_norm - initial_yaw_z_norm
        else:
            return 360 - initial_yaw_z_norm + current_yaw_z_norm
    else:
        if initial_yaw_z_norm >= current_yaw_z_norm:
            return initial_yaw_z_norm - current_yaw_z_norm
        else:
            return 360 - current_yaw_z_norm + initial_yaw_z_norm

