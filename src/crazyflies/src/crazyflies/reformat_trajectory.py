# This file is helper functions to reformat a trajectory from a ROS topic message to a 1D array for transport over 
# a ROS service and from the 1D array into a format for the crazyflie-lib-python high-level commander.
# I really don't like haveing to keep reformating the trajectory, but this makes the message fit the current archeticture 
# of the planner sending the message to the manager and the manager making a service call to the radio bridge.

from cflib.crazyflie.mem import Poly4D as CFPoly4D
from crazyflies.msg import Trajectory, Poly4D

FLOATS_PER_PIECE = 33  # 1 duration + 4 axes * 8 coefficients

def trajectory_topic_msg_to_array(trajectory_msg):
    """
    trajectory_msg: a crazyflie_trajectory_msgs/Trajectory message
    Returns: a flat list of floats, 33 per piece, in piece order.
    """
    array = []
    for piece in trajectory_msg.pieces:
        array.append(piece.duration)
        array.extend(piece.x)
        array.extend(piece.y)
        array.extend(piece.z)
        array.extend(piece.yaw)
    return array


def array_to_cflib_trajectory(array):
    """
    array: flat list/array of floats produced by trajectory_msg_to_array
           (length must be a multiple of 33)
    Returns: a list of cflib.crazyflie.mem.Poly4D objects, ready to assign
             to trajectory_mem.trajectory before write_data_sync().
    """
    if len(array) % FLOATS_PER_PIECE != 0:
        raise ValueError(
            f'Array length {len(array)} is not a multiple of '
            f'{FLOATS_PER_PIECE} (1 duration + 4x8 coefficients per piece)')
 
    trajectory = []
    for start in range(0, len(array), FLOATS_PER_PIECE):
        duration = array[start]
        x = list(array[start + 1: start + 9])
        y = list(array[start + 9: start + 17])
        z = list(array[start + 17: start + 25])
        yaw = list(array[start + 25: start + 33])
        trajectory.append(CFPoly4D(duration, _as_poly(x), _as_poly(y), _as_poly(z), _as_poly(yaw)))
 
    return trajectory

def cflib_trajectory_to_topic_msg(cflib_trajectory):
    """
    cflib_trajectory: list of cflib.crazyflie.mem.Poly4D objects
    Returns: a crazyflie_trajectory_msgs/Trajectory message
    """
    msg = Trajectory()
    for piece in cflib_trajectory:
        p = Poly4D()
        p.duration = piece.duration
        p.x = list(piece.x)
        p.y = list(piece.y)
        p.z = list(piece.z)
        p.yaw = list(piece.yaw)
        msg.pieces.append(p)
    return msg

def _as_poly(coeffs):
    """
    Wrap a list of 8 coefficients in cflib's Poly4D.Poly, which is what
    Poly4D.x / .y / .z / .yaw actually need to be (each must expose a
    `.values` attribute -- a plain list is NOT accepted here, even though
    it looks like it should be).
    """
    return CFPoly4D.Poly(list(coeffs))