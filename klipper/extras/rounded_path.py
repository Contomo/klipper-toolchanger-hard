# rounded paths for fast travel.
#
# Copyright (C) 2023  Viesturs Zarins <viesturz@gmail.com>

# Aimed to optimize travel paths by minimizing speed changes for sharp corners.
# Supports arbitrary paths in XYZ.
# Each corner is rounded to a maximum deviation distance of D.
# Since each corner depends on the next one, the chain needs to end with an R=0
# command to flush pending moves.
# Coordinates created by this are converted into G0 commands.

# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import logging
EPSILON = 0.001
EPSILON_ANGLE = 0.001

try:
    import numpy as np
except ModuleNotFoundError:
    np = None

class ControlPoint:
    def __init__(self, x, y, z, d, f):
        self.vec = [x,y,z]
        self.f = f
        self.maxd = d
        self.angle = 0.0
        self.len = 0.0 # distance to the previous point
        # max distance of the rounding from the corner based on D and angle.
        self.lin_d = 0.0
        self.lin_d_to_r = 0.0

# Some basic vector math
def _vecto(f: ControlPoint, t: ControlPoint)->list:
    return [t.vec[i]-f.vec[i] for i in range(3)]

def _vadd(f: list, t: list) ->list:
    return [f[i]+ t[i] for i in range(3)]

def _vmul(f:list, n) ->list:
    return [f[i] * n for i in range(3)]

def _cross(vp: list, vn: list) -> list:
    return [vp[1] * vn[2] - vp[2] * vn[1], vp[2] * vn[0] - vp[0] * vn[2],
           vp[0] * vn[1] - vp[1] * vn[0]]

def _vdist(v0: list, v1:list) -> float:
    return math.hypot(v0[0]-v1[0], v0[1]-v1[1], v0[2]-v1[2])

def _vnorm(vec: list) -> list:
    invlen = 1.0/math.hypot(*vec)
    return [x*invlen for x in vec]

def _vangle(vec1: list, vec2: list) -> float:
    crossx = vec1[1] * vec2[2] - vec1[2] * vec2[1]
    crossy = vec1[2] * vec2[0] - vec1[0] * vec2[2]
    crossz = vec1[0] * vec2[1] - vec1[1] * vec2[0]
    cross = math.hypot(crossx, crossy, crossz)
    dot = vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]
    return math.atan2(cross, dot)

def _vrot(vec: list, angle, axis: list) -> list:
    # Axis needs to be normalized
    # https://en.wikipedia.org/wiki/Rotation_matrix
    s = math.sin(angle)
    c = math.cos(angle)
    t = 1 - c
    return [
        vec[0] * (t * axis[0] ** 2 + c) + vec[1] * (t * axis[0] * axis[1] - s * axis[2]) + vec[2] * (t * axis[0] * axis[2] + s * axis[1]),
        vec[0] * (t * axis[0] * axis[1] + s * axis[2]) + vec[1] * (t * axis[1] ** 2 + c) + vec[2] * (t * axis[1] * axis[2] - s * axis[0]),
        vec[0] * (t * axis[0] * axis[2] - s * axis[1]) + vec[1] * (t * axis[1] * axis[2] + s * axis[0]) + vec[2] * (t * axis[2] ** 2 + c)
    ]

def _vrot_transform(angle: float, axis: list) -> list:
    # Axis needs to be normalized
    # https://en.wikipedia.org/wiki/Rotation_matrix
    s = math.sin(angle)
    c = math.cos(angle)
    t = 1 - c
    return [(t * axis[0] ** 2 + c), (t * axis[0] * axis[1] - s * axis[2]), (t * axis[0] * axis[2] + s * axis[1]),
        (t * axis[0] * axis[1] + s * axis[2]), (t * axis[1] ** 2 + c), (t * axis[1] * axis[2] - s * axis[0]),
        (t * axis[0] * axis[2] - s * axis[1]), (t * axis[1] * axis[2] + s * axis[0]), (t * axis[2] ** 2 + c)]

def _vtransform(vec: list, transform: list) -> list:
    return [vec[0] * transform[0] + vec[1]*transform[1] + vec[2]* transform[2],
            vec[0] * transform[3] + vec[1] * transform[4] + vec[2] * transform[5],
            vec[0] * transform[6] + vec[1] * transform[7] + vec[2] * transform[8]]

class RoundedPath:
    _ALGO_MAP ={'fillet':   'fillet',  'bezier':  'bezier',
               "'fillet'":  'fillet', "'bezier'": 'bezier'}
    buffer: list[ControlPoint]
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mm_per_arc_segment   = config.getfloat('resolution', 1.0, above=0.0)
        self.angle_resolution_deg = config.getfloat('angle_resolution', 1.0, above=0.0, maxval=180.0)
        self.log                  = config.getboolean('logging', False)
        self.algorithm            = config.getchoice('algorithm', self._ALGO_MAP, 'bezier')
        self.reset_on_mismatch    = config.getboolean('reset_on_mismatch', True)
        if self.algorithm == 'bezier' and np is None:
            raise config.error("Choice 'bezier' for option 'algorithm' in section 'rounded_path' requires 'numpy' to be installed." \
                                "(install numpy or switch to fillet)")
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.gcode = self.printer.lookup_object('gcode')
        self.G0_params = {}
        self.G0_cmd = self.gcode.create_gcode_command("G0", "G0", self.G0_params)
        self.real_G0 = self.gcode_move.cmd_G1
        self.gcode.register_command("ROUNDED_G0", self.cmd_ROUNDED_G0)
        self.buffer = []
        self.lastg0 = []

        if config.getboolean('replace_g0', False):
            self.gcode.register_command("G0", None)
            self.gcode.register_command("G0", self.cmd_ROUNDED_G0)

        self.printer.register_event_handler("gcode:command_error", self._handle_command_error)

    def _handle_command_error(self):
        self.buffer = []

    def cmd_ROUNDED_G0(self, gcmd):
        d = gcmd.get_float("D", 0.0)
        if d <= 0.0 and len(self.buffer) < 2:
            self.real_G0(gcmd)
            return
        gcodestatus = self.gcode_move.get_status()
        if not gcodestatus['absolute_coordinates']:
            raise gcmd.error("ROUNDED_G0 does not support relative move mode")
        currentPos = gcodestatus['gcode_position']
        if len(self.buffer) == 0:
            # Initialize with currentPos and radius = 0.
            self.buffer.append(ControlPoint(x = currentPos[0], y = currentPos[1], z = currentPos[2], d = 0.0, f = 0.0))
        else:
            origin = self.buffer[0].vec
            if _vdist(currentPos, origin) > EPSILON:
                if self.reset_on_mismatch:
                    self.buffer.clear()
                raise gcmd.error("ROUNDED_G0 - current position changed since previous command, the last ROUNDED_G0 before other moves needs to be with D=0")
            last = self.buffer[-1]
            currentPos = last.vec

        self._lineto(ControlPoint(x = gcmd.get_float("X", currentPos[0]),
                                  y = gcmd.get_float("Y", currentPos[1]),
                                  z = gcmd.get_float("Z", currentPos[2]),
                                  f = gcmd.get_float("F", 0.0),
                                  d = d))

    def _lineto(self, pos):
        self.buffer.append(pos)
        if len(self.buffer) >= 3:
            self._calculate_corner(self.buffer[-2], self.buffer[-3], self.buffer[-1])

        if len(self.buffer) >= 2 and self.buffer[-1].maxd <= 0.0:
            self._calculate_zero_corner(self.buffer[-1], self.buffer[-2])
            # zero max offset, flush everything.
            self._flush_buffer(len(self.buffer) -2)
            self._g0(self.buffer[-1])
            self.buffer.clear()
        elif len(self.buffer) >= 4 and self.buffer[-3].lin_d + self.buffer[-2].lin_d <= self.buffer[-2].len:
            # max offsets don't overlap, flush everything, but the last segment.
            self._flush_buffer(len(self.buffer) - 3)

    # Computes the max curve start offset along the edge based on max distance.
    def _calculate_corner(self, c:ControlPoint, v1:ControlPoint, v2:ControlPoint):
        vec1 = _vecto(c, v1)
        vec2 = _vecto(c, v2)
        c.len = math.hypot(*vec1)
        c.angle = _vangle(vec1, vec2)
        if abs(c.angle) < EPSILON_ANGLE or math.pi - abs(c.angle) < EPSILON_ANGLE:
            # too close of an angle - do not bother
            return
        sina2 = math.sin(c.angle / 2)
        tana2 = math.tan(c.angle/2)
        radius = c.maxd * sina2 / (1-sina2)
        c.lin_d_to_r = tana2
        c.lin_d = radius/tana2

    def _calculate_zero_corner(self, c:ControlPoint, vp:ControlPoint):
        vec1 = _vecto(c, vp)
        c.len = math.hypot(*vec1)
        c.angle = 0

    def _flush_buffer(self, num_segments):
        if num_segments <= 0:
            return
        if len(self.buffer) < 2:
            self.buffer.clear()
            return
        if len(self.buffer) == 2:
            self._g0(self.buffer[-1])
            self.buffer.clear()
            return

        self._deconflict_lin_d(num_segments+1)

        for i in range(num_segments):
            self._arc(self.buffer[i+1], self.buffer[i],self.buffer[i+2])

        self.buffer = self.buffer[num_segments:]
        # Update where we finished
        self.buffer[0].vec = self.lastg0

    def _deconflict_lin_d(self, num_segments):
        order = [i+1 for i in range(num_segments)]
        order = sorted(order, key=lambda a: self.buffer[a].len)
        # Process segments, shortest first
        for i in order:
            p0 = self.buffer[i-1]
            p1 = self.buffer[i]
            missingd = p1.lin_d + p0.lin_d - p1.len
            if missingd <= 0:
                continue

            # first try to reduce the biggest radius
            r0 = p0.lin_d * p0.lin_d_to_r
            r1 = p1.lin_d * p1.lin_d_to_r
            if r0 > r1:
                missingr0 = missingd * p0.lin_d_to_r + EPSILON
                r0 = max(r1, r0 - missingr0)
                p0.lin_d = r0 / p0.lin_d_to_r
            elif r1 > r0:
                missingr1 = missingd * p1.lin_d_to_r + EPSILON
                r1 = max(r0, r1 - missingr1)
                p1.lin_d = r1 / p1.lin_d_to_r
            missingd = p1.lin_d + p0.lin_d - p1.len
            if missingd <= 0:
                continue
            if p0.lin_d_to_r <= 0.0 or p1.lin_d_to_r <= 0.0:
                # should never happen, just to be safe, floating points are tricky
                p0.lin_d = 0
                p1.lin_d = 0
                continue
            # that was not enough, reduce both proportionally
            missingr_shared = missingd / (1.0 / p0.lin_d_to_r + 1.0 / p1.lin_d_to_r)
            p0.lin_d = max(0.0, p0.lin_d - missingr_shared / p0.lin_d_to_r)
            p1.lin_d = max(0.0, p1.lin_d - missingr_shared / p1.lin_d_to_r)

    def _arc(self, c: ControlPoint, p: ControlPoint, n: ControlPoint):
        if self.algorithm == 'bezier':
            self._arc_bezier(c, p, n)
        else:
            self._arc_fillet(c, p, n)

    def _arc_fillet(self, c: ControlPoint, p: ControlPoint, n: ControlPoint):
        radius = c.lin_d * c.lin_d_to_r
        angle = abs(c.angle)
        if angle <= EPSILON or radius <= EPSILON:
            self._g0(c)
            return
        path_length = radius * angle
        angle_step = math.radians(self.angle_resolution_deg)
        angle_segments = max(1, math.ceil(angle / angle_step))
        max_length_segments = max(1, math.floor(path_length / self.mm_per_arc_segment))
        if angle_segments <= max_length_segments:
            num_segments = angle_segments
        else:
            num_segments = max_length_segments
        if self.log:
            logging.info(
                "rounded_path fillet: angle=%.3fdeg radius=%.4f mm segments=%d "
                "(angle_segments=%d length_limit=%d)",
                math.degrees(angle), radius, num_segments,
                angle_segments, max_length_segments,
            )
        vp = _vnorm(_vecto(c, p))
        vn = _vnorm(_vecto(c, n))
        rotaxis = _vnorm(_cross(vp, vn))
        start = _vadd(c.vec, _vmul(vp, c.lin_d))
        spoke = _vmul(_vrot(vp, math.pi / 2.0, rotaxis), -radius)
        center = _vadd(start, _vmul(spoke, -1.0))

        # We are rotating counter the segment rotation.
        rot_transform = _vrot_transform(-c.angle / num_segments, rotaxis)
        rotspoke = spoke
        self._g0p(c, _vadd(center, rotspoke))
        for _ in range(num_segments):
            rotspoke = _vtransform(rotspoke, rot_transform)
            self._g0p(c, _vadd(center, rotspoke))

    def _arc_bezier(self, c: ControlPoint, p: ControlPoint, n: ControlPoint):
        if np is None:
            raise RuntimeError("NumPy is required for bezier algorithm")
        radius = c.lin_d * c.lin_d_to_r
        angle = abs(c.angle)
        if angle <= EPSILON or radius <= EPSILON:
            self._g0(c)
            return
        path_length = radius * angle
        angle_step = math.radians(self.angle_resolution_deg)
        angle_segments = max(1, math.ceil(angle / angle_step))
        max_length_segments = max(1, math.floor(path_length / self.mm_per_arc_segment))
        if angle_segments <= max_length_segments:
            num_segments = angle_segments
        else:
            num_segments = max_length_segments
        vp = _vnorm(_vecto(c, p))
        vn = _vnorm(_vecto(c, n))
        rotaxis = _vnorm(_cross(vp, vn))

        start = _vadd(c.vec, _vmul(vp, c.lin_d))
        spoke = _vmul(_vrot(vp, math.pi / 2.0, rotaxis), -radius)
        center = _vadd(start, _vmul(spoke, -1.0))

        # Convert tangential points into a quadratic Bezier through the corner
        np_n = np.array(vn) * c.lin_d  # next tangential point (vector from c)
        np_p = np.array(vp) * c.lin_d  # previous tangential point (vector from c)
        np_c = np.array(c.vec)         # corner itself
        straight_len = math.hypot(*(np_n - np_p))
        straight_segments = max(1, math.floor(straight_len / self.mm_per_arc_segment))
        if self.log:
            logging.info(
                "rounded_path bezier: angle=%.3fdeg radius=%.4f mm segments=%d "
                "(angle_segments=%d length_limit=%d straight_limit=%d)",
                math.degrees(angle), radius, num_segments,
                angle_segments, max_length_segments, straight_segments,
            )
        # Cap bezier sampling to the chosen segment count to avoid excessive points
        n_pts = max(2, num_segments + 1)
        bcurve = _Bezier.bezier_curve([np_p + np_c, np_c, np_n + np_c], n=n_pts)

        # Emit: first move to entry tangent, then follow Bezier in reverse so
        # that curves are generated from incoming to outgoing segment order.
        self._g0p(c, _vadd(center, spoke))
        for point in reversed(bcurve):
            self._g0p(c, point)

    def _g0(self, p: ControlPoint):
        self._g0p(p, p.vec)

    def _g0p(self, p: ControlPoint, vec: list):
        self.G0_params["X"]=vec[0]
        self.G0_params["Y"]=vec[1]
        self.G0_params["Z"]=vec[2]
        if p.f > 0.0:
            self.G0_params['F'] = p.f
        else:
            self.G0_params.pop('F', None)
        self.lastg0 = vec
        self.real_G0(self.G0_cmd)


# This module has been adapted from code written by Ingo Donasch <ingo@donasch.net>
# Sourced from https://github.com/idonasch/klipper-toolchanger/blob/bezier/klipper/extras/bezier_path.py
class _Bezier:
    """Utility class for n-point Bezier curves."""

    @staticmethod
    def _comb(n: int, k: int) -> float:
        """N choose k"""
        return math.factorial(n) / math.factorial(k) / math.factorial(n - k)

    @classmethod
    def _bernstein_poly(cls, i: int, n: int, t):
        """The Bernstein polynomial of n, i as a function of t"""
        return cls._comb(n, i) * (t ** (n - i)) * (1 - t) ** i

    @classmethod
    def bezier_curve(cls, points, n=1000):
        """
           Given a set of control points, return the
           bezier curve defined by the control points.

           points should be a list of lists, or list of tuples
           such as [ [1,1,1], 
                     [2,3,2], 
                     [4,5,4], ..[Xn, Yn, Zn] ]
            n is the number of points at which to return the curve, defaults to 1000

            See http://processingjs.nihongoresources.com/bezierinfo/
        """
        if np is None:
            raise RuntimeError("NumPy is required for 'bezier' algorithm but not available.")
        
        nPoints = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])
        zPoints = np.array([p[2] for p in points])

        t = np.linspace(0.0, 1.0, n)

        polynomial_array = np.array(
            [cls._bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)]
        )

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)
        zvals = np.dot(zPoints, polynomial_array)

        return np.transpose([xvals, yvals, zvals]).tolist()


def load_config(config):
    return RoundedPath(config)
