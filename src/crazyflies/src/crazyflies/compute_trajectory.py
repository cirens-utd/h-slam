"""
Convert a list of (x, y, z, yaw) waypoints into a Poly4D trajectory that
flies smoothly THROUGH each intermediate waypoint (no stopping), using a
clamped cubic spline per axis.

How it works:
    For each axis (x, y, z, yaw) independently, a cubic spline is fit
    through all waypoint values with the given segment durations. A cubic
    spline guarantees position, velocity, AND acceleration are continuous
    across every internal waypoint -- so the Crazyflie keeps moving
    smoothly through them rather than braking to a stop.

    The only place velocity is forced to zero is the very first and very
    last waypoint ("clamped" boundary conditions), which is what you want
    for takeoff/hover and landing/hover.

    Each cubic segment (4 coefficients) is embedded directly into the
    8-coefficient Poly4D format used by the high-level commander -- the
    top 4 coefficients (t^4..t^7) are simply zero.

No external dependencies (numpy, scipy) are required; the tridiagonal
spline system is solved with a plain Thomas algorithm implementation.
"""

import math

from cflib.crazyflie.mem import Poly4D

def _solve_tridiagonal(a, b, c, d):
    """
    Thomas algorithm for a tridiagonal system.
    a: sub-diagonal, a[i] multiplies x[i-1] in row i (a[0] unused)
    b: main diagonal
    c: super-diagonal, c[i] multiplies x[i+1] in row i (c[-1] unused)
    d: right-hand side
    Returns solution vector x, same length as d.
    """
    n = len(d)
    b = list(b)
    d = list(d)
    for i in range(1, n):
        w = a[i] / b[i - 1]
        b[i] -= w * c[i - 1]
        d[i] -= w * d[i - 1]
    x = [0.0] * n
    x[-1] = d[-1] / b[-1]
    for i in range(n - 2, -1, -1):
        x[i] = (d[i] - c[i] * x[i + 1]) / b[i]
    return x


def _clamped_cubic_spline_second_derivatives(y, h, v0=0.0, vn=0.0):
    """
    Solve for the spline's second derivatives (M) at every waypoint,
    given waypoint values y (length n+1), segment durations h (length n),
    and prescribed first-derivative (velocity) boundary conditions v0/vn.
    """
    n = len(h)          # number of segments
    size = n + 1        # number of waypoints / unknown M values

    a = [0.0] * size
    b = [0.0] * size
    c = [0.0] * size
    d = [0.0] * size

    # Clamped boundary condition at the first waypoint
    b[0] = 2 * h[0]
    c[0] = h[0]
    d[0] = 6 * ((y[1] - y[0]) / h[0] - v0)

    # Interior continuity equations
    for i in range(1, n):
        a[i] = h[i - 1]
        b[i] = 2 * (h[i - 1] + h[i])
        c[i] = h[i]
        d[i] = 6 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])

    # Clamped boundary condition at the last waypoint
    a[n] = h[n - 1]
    b[n] = 2 * h[n - 1]
    d[n] = 6 * (vn - (y[n] - y[n - 1]) / h[n - 1])

    return _solve_tridiagonal(a, b, c, d)


def _spline_segment_coeffs(y, h, M):
    """
    Given waypoint values y, segment durations h, and second derivatives M
    (from _clamped_cubic_spline_second_derivatives), return per-segment
    cubic coefficients (c0, c1, c2, c3) such that, with t measured from
    the start of the segment:
        value(t) = c0 + c1*t + c2*t^2 + c3*t^3
    """
    n = len(h)
    segments = []
    for i in range(n):
        c0 = y[i]
        c2 = M[i] / 2.0
        c3 = (M[i + 1] - M[i]) / (6.0 * h[i])
        c1 = ((y[i + 1] - y[i]) / h[i]
              - h[i] * (2 * M[i] + M[i + 1]) / 6.0)
        segments.append((c0, c1, c2, c3))
    return segments


def _unwrap(values, period=2 * math.pi):
    """
    Unwrap a sequence of angles so consecutive waypoints are connected by
    the shortest path, avoiding e.g. a jump from 350 deg to 10 deg being
    treated as a -340 deg turn. Set period=360.0 if you're using degrees.
    """
    unwrapped = [values[0]]
    for v in values[1:]:
        prev = unwrapped[-1]
        diff = v - prev
        while diff > period / 2:
            v -= period
            diff = v - prev
        while diff < -period / 2:
            v += period
            diff = v - prev
        unwrapped.append(v)
    return unwrapped


def waypoints_to_smooth_trajectory(waypoints, segment_time=2.0,
                                    unwrap_yaw=True, yaw_period=2 * math.pi):
    """
    Args:
        waypoints: list of (x, y, z, yaw) tuples, in meters and radians
                   (pass yaw_period=360.0 if you're using degrees).
        segment_time: a single float (seconds per segment, applied to all
                      segments) or a list of floats with length
                      len(waypoints) - 1, one duration per segment.
        unwrap_yaw: if True, adjusts yaw values so consecutive waypoints
                    are connected via the shortest angular path instead
                    of wrapping the long way around.
        yaw_period: the full-turn period of your yaw values (2*pi for
                    radians, 360.0 for degrees). Only used if
                    unwrap_yaw=True.

    Returns:
        A list of Poly4D objects, ready to assign to
        trajectory_mem.trajectory before calling write_data_sync().
    """
    if len(waypoints) < 2:
        raise ValueError('Need at least 2 waypoints to build a trajectory')

    n_segments = len(waypoints) - 1

    if isinstance(segment_time, (int, float)):
        h = [float(segment_time)] * n_segments
    else:
        h = list(segment_time)
        if len(h) != n_segments:
            raise ValueError(
                f'segment_time list must have {n_segments} entries '
                f'(len(waypoints) - 1), got {len(h)}')

    xs = [w[0] for w in waypoints]
    ys = [w[1] for w in waypoints]
    zs = [w[2] for w in waypoints]
    yaws = [w[3] for w in waypoints]

    if unwrap_yaw:
        yaws = _unwrap(yaws, period=yaw_period)

    def axis_to_poly4d_coeffs(vals):
        M = _clamped_cubic_spline_second_derivatives(vals, h, v0=0.0, vn=0.0)
        segs = _spline_segment_coeffs(vals, h, M)
        # Pad each cubic (4 coeffs) out to the 8 coeffs Poly4D expects
        return [[c0, c1, c2, c3, 0.0, 0.0, 0.0, 0.0] for (c0, c1, c2, c3) in segs]

    x_coeffs = axis_to_poly4d_coeffs(xs)
    y_coeffs = axis_to_poly4d_coeffs(ys)
    z_coeffs = axis_to_poly4d_coeffs(zs)
    yaw_coeffs = axis_to_poly4d_coeffs(yaws)

    trajectory = []
    for i in range(n_segments):
        trajectory.append(
            Poly4D(h[i], x_coeffs[i], y_coeffs[i], z_coeffs[i], yaw_coeffs[i]))

    return trajectory


if __name__ == '__main__':
    # Example: fly a smooth loop through 5 waypoints without stopping at
    # the corners, only coming to rest at the first/last point.
    waypoints = [
        (0.0, 0.0, 1.0, 0.0),
        (1.0, 0.0, 1.0, 0.0),
        (1.0, 1.0, 1.0, 1.5708),
        (0.0, 1.0, 1.0, 3.1416),
        (0.0, 0.0, 1.0, 0.0),
    ]

    traj = waypoints_to_smooth_trajectory(waypoints, segment_time=2.0)
    total_duration = sum(p.duration for p in traj)
    print(f'Built {len(traj)} segments, total duration {total_duration:.1f}s')

    # Usage with cflib:
    #
    # from cflib.crazyflie.mem import MemoryElement
    #
    # trajectory_id = 1
    # trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    # trajectory_mem.trajectory = traj
    # if trajectory_mem.write_data_sync():
    #     cf.high_level_commander.define_trajectory(
    #         trajectory_id, 0, len(traj))
    #     cf.high_level_commander.start_trajectory(
    #         trajectory_id, relative_position=True)
    #     time.sleep(total_duration)