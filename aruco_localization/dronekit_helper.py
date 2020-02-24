from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import sys, termios, tty, os, time, math
import numpy as np


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def set_attitude(vehicle, roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """

    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """

    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        0,
        # Target system
        0,
        # Target component
        0b00000000,
        # Type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle),
        # Quaternion
        0,
        # Body roll rate in radian
        0,
        # Body pitch rate in radian
        math.radians(yaw_rate),
        # Body yaw rate in radian
        thrust)
    # Thrust
    vehicle.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(modf[0])

        # Send command to vehicle on 1 Hz cycle
        for x in range(0, int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def pid_control(x_pos, y_pos, z_pos):
    """
    Input:
    Takes in the (x,y,z) positions of the drone from the aruco code and sets the row and pitch of the
    """
    init_bias = 5
    x_summand = 0
    y_summand = 0
    z_summand = 0
    initial_error_x = 0
    initial_error_y = 0
    initial_error_z = 0
    k_p = 5
    k_i = 7
    k_d = 10
    iter_time = 0.05
    target_x, target_y,target_z = (0, 0, 0)
    target_coords = (target_x, target_y, target_z)

    x_err = target_coords[0] - x_pos
    y_err = target_coords[1] - y_pos
    z_err = target_coords[2] - z_pos

    x_deriv = (x_err - initial_error_x) / (iter_time)
    y_deriv = (y_err - initial_error_y) / (iter_time)
    z_deriv = (z_err - initial_error_z) / (iter_time)

    x_summand = x_summand + (x_err * iter_time)
    y_summand = y_summand + (y_err * iter_time)
    z_summand = z_summand + (z_err * iter_time)

    x_out = init_bias + k_i * x_err + k_d * x_deriv + k_i * x_summand
    y_out = init_bias + k_i * y_err + k_d * y_deriv + k_i * y_summand
    z_out = init_bias + k_i * z_err + k_d * z_deriv + k_i * z_summand
    initial_error_x = x_err
    initial_error_y = y_err
    initial_error_z = z_err
    return (x_out, y_out, z_out)
