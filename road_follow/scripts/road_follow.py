#!/usr/bin/env python3
"""Drive a lap of the carreadores by eye, and come back to the middle.

    ros2 run road_follow road_follow
    ros2 run road_follow road_follow --ros-args -p speed:=2.0
    ros2 run road_follow road_follow --ros-args -p laps:=1

Run it in a second shell inside the *running* container, so it shares the ROS
graph with the simulator:

    docker exec -it ros2-ws-dev bash
    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor \
        world:=citrus_farm.sdf                  # first shell
    ros2 run road_follow road_follow             # second shell

The tractor leaves the crossroads at the centre of the world heading east,
takes the main carreador out to the east boundary, turns left, runs up the
boundary and round its corner, turns left again onto the carreador that comes
back down the middle of the property, and arrives at the crossroads it started
from. About 1.1 km, a quarter of the network, and then it does it again.

**The steering is the camera and nothing else.** Every metre of every straight
and every curve is driven by finding the road in /camera_front/image_raw and
steering to put the machine down the middle of it. Odometry and the receiver
are read, but only to answer the one question the road cannot: at a junction
where two carreadores cross, which of them is mine. That is a map question,
not a perception one, and pretending the camera answers it would be a lie
dressed as a demo. So the route below carries the junctions and the machine's
own position says when one has arrived; the camera does the driving.

WHAT IT FOLLOWS. The running surface, and its two edges. A carreador here is
12 m of bare earth with 7 m of graded track down the middle of it, and that
inner strip is a few counts lighter than the shoulders either side: its edges
sit at +/-3.5 m and their midpoint is the centreline. Nothing is painted on
the road and nothing is drawn on it that a road would not have -- a white line
was tried and taken out because nobody paints a farm track, and a pair of
wheel ruts after it, because hard edged stripes 2.1 m apart read as paint too
and matched the bitola of no machine on the property. What is left is the road
itself, and it turns out to be the better target.

Finding the edges needs no threshold that depends on the weather. The step
from surface to shoulder is five to eight counts of red, and the surface
itself changes with the light and the slope -- 176 counts at 8 m ahead in this
world, 156 at 14 m -- so the row is smoothed over five pixels and compared
against its own median across a 10 m window. The surface is most of that
window by construction, so the median IS the surface, and anything a few
counts under it is shoulder.

Two bands of the image, one 6 m ahead and one 14 m, chosen by *distance* and
not by row: which pixel row that is comes from the camera's own intrinsics in
/camera_front/camera_info and its height in tf, so a camera moved in the
description moves these bands with it.

A pair of edges is better than one mark, and that is the point of using them.
Two edges 7 m apart either side of centre give a measurement that is
symmetric: the midpoint is the centreline whether or not the machine is square
to the road, and a junction, where the surface of the road that joins widens
the strip past 7 m, fails the spacing test and is rejected rather than
followed. Where only one edge survives the other still fixes the centre, 3.5 m
to its side, on a tighter gate. Both gone means the surface ran past both ends
of the window, which is a crossing or a painted number lying over the road,
and a crossing is where the route is steering anyway; the follower holds the
curvature it had, which on a straight is right and on the boundary's 24 m arc
is exactly right.

HOW IT STEERS. Pure pursuit on the far band — curvature 2*y/(L^2 + y^2) to put
the machine on the road L metres ahead — plus a small term on the near error so
a steady sideways offset is actually removed rather than tracked in parallel.
The result is a curvature, and what goes out is angular.z = speed x curvature,
for the reason teleop_key gives at length: an Ackermann machine reads
angular.z as a yaw rate, and a yaw rate at one speed is a different circle at
another.

WHAT IT CANNOT DO. It will not find the road again on its own if it loses it
for good — it slows, holds, and after `lost_timeout` stops and says so, which
is the honest failure for a machine with no map and one camera.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from tf2_ros import Buffer, TransformListener

# The lap, as legs. Each one is a straight run of carreador: the direction it
# is driven in, the point where it ends, and what happens there.
#
#   "left"  turn left onto the next carreador, on an arc of the given radius
#   "coast" nothing to command -- the road itself curves here and the camera
#           drives it; the entry only exists so that the leg after it measures
#           its distance along its own direction and not along this one
#
# This is citrus_farm: a 560 m square with its boundary road at +/-280, the
# main carreador along y = 0, the cross carreador along x = 0, and the
# boundary's corners bladed round at 24 m from x = 256.
#
# Where a turn begins is geometry, not taste: an arc of radius R that leaves
# one carreador at right angles lands on the next one R metres along it, so
# the arc onto the boundary at x = 280 has to begin at x = 280 - R. Get that
# wrong and the machine lands beside the road and has to crab back onto it.
# THE ROUTES ARE IN METRES FROM WHERE THE MACHINE STARTED, not from the world
# origin, and the two are the same thing only because every route here is
# driven from the world origin. The fix's datum is the first thing it sees,
# which is the machine standing still at the start (see _on_fix), so a vehicle
# spawned 40 m west of the origin turns 40 m early -- which is exactly what
# happened the first time docs/simulation.mp4 was shot on "origin_bend", and
# the tractor left the carreador and drove into the talhao at a confident
# 3 m/s. Start on the origin, or write the route in the frame you start in.
ROUTES = {
    # The lap, and the default: out to the boundary, round its corner, back
    # down the middle of the property to the crossroads it started from.
    "lap": [
        # direction     the leg ends at    there        radius
        ((1.0, 0.0), (268.0, 0.0), "left", 12.0),      # main, east to the boundary
        ((0.0, 1.0), (280.0, 256.0), "coast", 24.0),   # boundary, north to its corner
        ((-1.0, 0.0), (12.0, 280.0), "left", 12.0),    # boundary, west to the cross road
        ((0.0, -1.0), (0.0, 6.0), "left", 6.0),        # cross road, south to the crossroads
    ],
    # No route at all: follow whatever carreador the machine is standing on
    # until it is stopped. "run" is a leg that never ends, so no junction is
    # ever reached and nothing is ever turned at -- which is what is wanted on
    # a road that only goes one way, like the 60 degree road through the
    # north-east talhao, and what the camera work in docs/film uses.
    "follow": [((1.0, 0.0), (0.0, 0.0), "run", 0.0)],
    # East from the centre of the world along the main carreador, then off it
    # onto the 30 degree road that leaves for the south-east talhao. A short
    # straight through the crossroads and then a bend, which is what
    # docs/simulation.mp4 is built around.
    #
    # The arc begins 5.4 m before the junction and not at it: an arc of radius
    # R turning through an angle eats R*tan(half the angle) off the road at
    # each end, which at 20 m and 30 degrees is 5.36. Begin the turn at the
    # junction itself and the machine finishes it five metres into the talhao.
    "origin_bend": [
        ((1.0, 0.0), (42.7, 0.0), "right", 20.0),          # the main, east
        ((0.866, -0.5), (0.0, 0.0), "run", 0.0),           # the 30 degree road
    ],
}
# The last arc is the tight one, and that is the point of it. Rounding the
# crossroads at 6 m brings the machine back within 2.5 m of the world origin
# it started from; at the 12 m the other turns use it would pass 5 m wide. An
# Ackermann vehicle cannot arrive at a point and leave it in another
# direction, so "back to the middle" is a corner taken close, not a dot hit.


class LineFollow(Node):
    def __init__(self):
        super().__init__("road_follow")

        def param(name, default):
            return self.declare_parameter(name, default).value

        # Speeds. The machine will do 4.2 m/s; a tractor on a carreador with a
        # camera for eyes does not, and neither does this.
        self.speed = param("speed", 3.0)
        self.bend_speed = param("bend_speed", 2.0)
        self.turn_speed = param("turn_speed", 1.2)
        self.accel = param("accel", 1.0)

        # Where the two bands look, in metres of ground ahead. The near one
        # is as close as the camera can see the ground -- 4.5 m at the bottom
        # of the frame -- and the far one as far as the road stays in frame
        # through the boundary's 24 m corner, which is about 15 m.
        self.near_d = param("near_distance", 6.0)
        self.far_d = param("far_distance", 14.0)

        # The running surface, from generate_agriculture_worlds.py: ROAD_W is
        # 7.0 m of graded track down the middle of 12 m of bare earth, so its
        # edges sit at +/-3.5 m and their midpoint is the centreline. `step`
        # is how far below the surface's own median a pixel has to fall to be
        # shoulder rather than surface -- the measured difference is 5 to 8
        # counts, against a surface that itself runs 176 at 8 m and 156 at 14,
        # so the test has to be relative and the threshold small. `window` is
        # how far either side of the last centre to look.
        self.surface_half = param("surface_half", 3.5)
        self.surface_step = param("surface_step", 3.0)
        self.window = param("search_window", 5.0)

        # Steering. min_radius is the machine's own, from params.xacro by way
        # of autonomous_tractor.gz.xacro: 0.60 rad of lock on a 2.15 m
        # wheelbase. tau is how fast the wheel is allowed to move, which is
        # what keeps a noisy centroid from becoming a shimmy.
        self.min_radius = param("min_radius", 3.2)
        self.near_gain = param("near_gain", 0.06)
        self.tau = param("steer_tau", 0.30)

        # The gate: how far the middle of the road may move between one frame
        # and the next and still be believed. Tracking, 1.0 m is really a
        # speed limit rather than a gate -- fifteen frames a second at 3 m/s
        # is 0.2 m of ground a frame, and the road does not move sideways
        # faster than that, so anything that does is a junction eating a verge
        # rather than a road going somewhere. Acquiring, having just come out
        # of a turn with no history at all, it has to be wide enough to find
        # the carreador the machine is standing on.
        self.gate_track = param("gate_track", 1.0)
        self.gate_acquire = param("gate_acquire", 6.0)
        self.reacquire_after = param("reacquire_after", 1.5)

        # Where the machine is, for the route only. Wheel odometry alone will
        # not do it: one lap of this circuit is 1107 m with three 90 degree
        # turns on it, and the odometry came back 9 m out -- 0.8%, which is an
        # ordinary number for a slipping Ackermann machine and quite enough to
        # start a turn in the wrong place by the third lap. The receiver on
        # the mast does not drift, so the two are fused the obvious way round:
        # odometry for smoothness between fixes, the fix for where that
        # smoothness is anchored. Its origin is the first thing it sees, which
        # is the machine standing at the centre of the world, so the route
        # below is in metres from there and no world's datum is written down.
        self.use_gnss = param("use_gnss", True)
        self.fix_samples = param("fix_samples", 25)
        self.fix_alpha = param("fix_alpha", 0.05)

        # Which of ROUTES to drive. "lap" is the circuit; "follow" just keeps
        # to the road it is on.
        self.route = ROUTES[param("route", "lap")]

        self.laps = param("laps", 0)             # 0 = until stopped
        # How long the machine may drive on held curvature before it gives up
        # and stops. It has to be longer than the longest honest blindness on
        # this circuit, which is the painted number on the boundary corner:
        # 7.8 m of pale board lying across the road, crossed at 1.2 m/s, is
        # six and a half seconds, and at six the lap stopped dead on it.
        self.lost_timeout = param("lost_timeout", 12.0)
        self.rate = param("rate", 30.0)

        self.pub = self.create_publisher(Twist, param("topic", "cmd_vel"), 10)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "camera_front/image_raw",
                                 self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, "camera_front/camera_info",
                                 self._on_info, sensor_qos)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(NavSatFix, "gnss/fix", self._on_fix, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # camera, filled in from camera_info and tf
        self.fx = self.fy = self.cx = self.cy = None
        self.cam_height = None

        # where we are
        self.pos = None                 # odometry plus the fix's correction
        self.odom_xy = None             # odometry alone, as published
        self.offset = (0.0, 0.0)        # what the fix says to add to it
        self.fix_origin = None          # (lat, lon) of the centre of the world
        self.fix_seen = []
        self.yaw = 0.0
        self.travelled = 0.0
        self.last_pos = None

        # What the camera last said, band by band. Each band tracks its own
        # stretch of road: gating the far band on what the near band sees
        # sounds tidier and is wrong, because the far band is looking at
        # ground the near one will not reach for a couple of seconds, and at a
        # junction that is exactly the ground where the verge is missing.
        self.track = {"near": None, "far": None}
        self.track_at = {"near": 0.0, "far": 0.0}
        self.seen_at = 0.0
        self.near_y = self.far_y = None
        self.wide = False

        # what we are doing
        self.leg = 0
        self.lap = 1
        self.state = "wait"
        self.curvature = 0.0
        self.commanded = 0.0
        self.out_speed = 0.0
        self.turn_from = 0.0
        self.turned = 0.0
        self.turn_sign = 0.0
        self.turn_sweep = math.pi / 2
        self.turn_radius = 0.0
        self.lap_start = None
        self.lap_length = 0.0
        self.closest = 1e9
        self.stopped_reason = None

    # -- what comes in ------------------------------------------------------
    def _on_info(self, msg):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f"camera: {msg.width}x{msg.height}, f = {self.fx:.1f} px, "
                f"centre ({self.cx:.1f}, {self.cy:.1f})")

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        here = (p.x, p.y)
        # Distance run comes from the odometry raw: it is a difference over
        # 30 ms and the fix's correction has no business in it.
        if self.last_pos is not None:
            step = math.dist(here, self.last_pos)
            self.travelled += step
            self.lap_length += step
        self.last_pos = self.odom_xy = here
        self.pos = (here[0] + self.offset[0], here[1] + self.offset[1])
        # How close the lap came to the middle of the world, which is the
        # thing the lap is for. Only once the machine has actually left:
        # a lap that starts at the origin is trivially 0.00 m from it, and
        # reporting that would be flattering itself.
        if self.lap_length > 50.0:
            self.closest = min(self.closest, math.hypot(*self.pos))

    def _on_fix(self, msg):
        """The receiver, turned into metres from where the machine started.

        The first `fix_samples` fixes are averaged with the machine standing
        still, and that average is the origin: the middle of the world, which
        is where sim.launch.py puts a vehicle. Everything after is an offset
        from it, in the world's own axes, because these worlds are laid out
        with ENU and no heading rotation.
        """
        if not self.use_gnss or math.isnan(msg.latitude):
            return
        if self.fix_origin is None:
            self.fix_seen.append((msg.latitude, msg.longitude))
            if len(self.fix_seen) >= self.fix_samples:
                lat = sum(f[0] for f in self.fix_seen) / len(self.fix_seen)
                lon = sum(f[1] for f in self.fix_seen) / len(self.fix_seen)
                # Metres per degree on WGS84 at this latitude. The usual
                # spherical shortcut is 0.4% out here, which over 280 m of
                # carreador is a metre of where a turn begins.
                phi = math.radians(lat)
                self.per_lat = (111132.92 - 559.82 * math.cos(2 * phi)
                                + 1.175 * math.cos(4 * phi))
                self.per_lon = (111412.84 * math.cos(phi)
                                - 93.5 * math.cos(3 * phi))
                self.fix_origin = (lat, lon)
                self.get_logger().info(
                    f"origem do GNSS fixada em {len(self.fix_seen)} amostras: "
                    f"{lat:.6f}, {lon:.6f}")
            return
        if self.odom_xy is None:
            return
        east = (msg.longitude - self.fix_origin[1]) * self.per_lon
        north = (msg.latitude - self.fix_origin[0]) * self.per_lat
        # A slow pull, not a jump. The fix carries a metre of noise and the
        # odometry carries none, so what is wanted from the fix is only its
        # lack of drift: at 5 Hz and 0.05 the correction has a four second
        # time constant and a sixth of the noise it is made of.
        want = (east - self.odom_xy[0], north - self.odom_xy[1])
        a = self.fix_alpha
        self.offset = (self.offset[0] + (want[0] - self.offset[0]) * a,
                       self.offset[1] + (want[1] - self.offset[1]) * a)

    def _row_for(self, distance):
        """The image row the ground `distance` ahead falls on.

        A camera looking level at height h sees the ground at distance d at
        an angle atan(h/d) below its axis, which is fy*h/d pixels below the
        principal point. No small angle approximation and no calibration by
        eye: both numbers come from the robot's own description.
        """
        return int(round(self.cy + self.fy * self.cam_height / distance))

    def _surface(self, row, lo, hi):
        """Where the graded surface starts and stops in a stretch of a row.

        Returns (start, stop) into the row, or None. The difference between
        the surface and the bare earth beside it is five to eight counts of
        red, which is real and small, so the row is smoothed first and then
        compared against its own median rather than against any number: the
        surface is most of the window by construction, so the median is the
        surface, and anything a few counts under it is shoulder.
        """
        red = row[lo:hi, 0].astype(np.float32)
        if red.size < 16:
            return None
        smooth = np.convolve(red, np.ones(5, np.float32) / 5.0, mode="same")
        # The median is meant to be the surface, and one thing in these worlds
        # is much brighter than the surface: the pale board a bend's angle is
        # painted on, 7.8 by 6.0 m and lying right across the road. Where it
        # covers part of the window it drags the median up with it and the
        # real surface falls on the wrong side of the threshold, so the median
        # is taken again over what is left once the board is excluded.
        level = np.median(smooth)
        keep = smooth <= level + 8.0
        if keep.any():
            level = np.median(smooth[keep])
        bright = (smooth >= level - self.surface_step) & keep
        if not bright.any():
            return None
        edges = np.flatnonzero(
            np.diff(np.concatenate(([0], bright.astype(np.int8), [0]))))
        runs = [(lo + a, lo + b) for a, b in zip(edges[0::2], edges[1::2])]
        return runs or None

    def _band(self, image, distance, band, now):
        """The middle of the carreador in one band, as a pixel column.

        Returns (column, nothing usable seen). The second is not quite a
        failure: on these roads it means the surface ran past both ends of the
        window, which happens at a crossing and over a painted number, and the
        caller holds rather than steers.
        """
        row_index = self._row_for(distance)
        if not 0 <= row_index < image.shape[0]:
            return None, False

        # Lost for long enough and this band forgets where the road was,
        # which is what lets it find one again after a turn.
        if now - self.track_at[band] > self.reacquire_after:
            self.track[band] = None
        tracking = self.track[band] is not None
        anchor = self.track[band] if tracking else self.cx

        scale = self.fx / distance                  # pixels per metre out there
        lo = max(0, int(anchor - self.window * scale))
        hi = min(image.shape[1], int(anchor + self.window * scale))
        runs = self._surface(image[row_index], lo, hi)
        if not runs:
            return None, True
        here = [r for r in runs if r[0] <= anchor < r[1]]
        start, stop = here[0] if here else max(runs, key=lambda r: r[1] - r[0])

        # An edge that lands on the end of the window is not an edge: it is
        # where the looking stopped.
        left = None if start <= lo else start
        right = None if stop >= hi else stop - 1

        half_px = self.surface_half * scale
        gate_px = (self.gate_track if tracking else self.gate_acquire) * scale
        if left is not None and right is not None:
            span = right - left
            if not 0.6 * 2 * half_px <= span <= 1.4 * 2 * half_px:
                return None, True
            u = (left + right) / 2.0
        elif left is not None:
            u = left + half_px
            gate_px = min(gate_px, 1.0 * scale)
        elif right is not None:
            u = right - half_px
            gate_px = min(gate_px, 1.0 * scale)
        else:
            return None, True

        if abs(u - anchor) > gate_px:
            return None, True
        self.track[band] = u
        self.track_at[band] = now
        return u, False

    def _on_image(self, msg):
        if self.fx is None or self.cam_height is None:
            return
        if msg.encoding != "rgb8":
            self.get_logger().warn(f"camera says {msg.encoding}, expected rgb8",
                                   once=True)
            return
        image = (np.frombuffer(msg.data, np.uint8)
                 .reshape(msg.height, msg.step)[:, :msg.width * 3]
                 .reshape(msg.height, msg.width, 3))

        now = time.monotonic()
        near_u, near_junction = self._band(image, self.near_d, "near", now)
        far_u, far_junction = self._band(image, self.far_d, "far", now)
        self.wide = near_junction or far_junction

        def lateral(u, distance):
            """Metres left of the machine's axis. u right of centre is right."""
            return (self.cx - u) * distance / self.fx

        if near_u is not None:
            self.near_y = lateral(near_u, self.near_d)
            self.seen_at = now
        else:
            self.near_y = None
        self.far_y = lateral(far_u, self.far_d) if far_u is not None else None

    # -- what goes out ------------------------------------------------------
    def _steer_from_road(self):
        """Curvature the camera is asking for, or None if it cannot see."""
        if self.far_y is None and self.near_y is None:
            return None
        # Pure pursuit on the far band: the arc from here through a point y
        # metres to the side, L metres ahead, has curvature 2y/(L^2 + y^2),
        # which is exact and not an approximation -- put the point on a circle
        # through the machine tangent to its axis and the formula returns that
        # circle's own curvature.
        kappa = 0.0
        if self.far_y is not None:
            L = self.far_d
            kappa += 2.0 * self.far_y / (L * L + self.far_y * self.far_y)
        # and a gentle pull on the near error, which is what takes out an
        # offset the look-ahead alone would happily drive parallel to.
        if self.near_y is not None:
            kappa += self.near_gain * self.near_y
        limit = 1.0 / self.min_radius
        return max(-limit, min(limit, kappa))

    def _remaining(self):
        """Metres of this leg left, measured along the leg's own direction.

        Along the leg's direction and not the machine's, because one leg
        contains the boundary's own 24 m corner: measured along a heading that
        is itself swinging through 90 degrees, the distance to the end of the
        leg would run backwards halfway round.
        """
        (dx, dy), (ex, ey), action, _ = self.route[self.leg]
        if action == "run":
            return float("inf")          # a leg that never ends
        return (ex - self.pos[0]) * dx + (ey - self.pos[1]) * dy

    def _sweep_to_next_leg(self):
        """How far this turn goes, from the two legs it joins.

        Taken from the route's own directions rather than written beside the
        turn, because a number written twice is a number that will disagree
        with itself -- and this one did. It was not written at all, in fact:
        every turn ran until the machine had swung 90 degrees, which is right
        for three sides of a square and wrong for the 30 degree road off the
        main. Asked for 30, the tractor turned 90, left the carreador and
        drove into the talhao at a confident 3 m/s.
        """
        here = self.route[self.leg][0]
        nxt = self.route[(self.leg + 1) % len(self.route)][0]
        sweep = math.atan2(nxt[1], nxt[0]) - math.atan2(here[1], here[0])
        return math.atan2(math.sin(sweep), math.cos(sweep))

    def _next_leg(self):
        self.leg += 1
        if self.leg < len(self.route):
            return
        self.leg = 0
        self.lap += 1
        self.get_logger().info(
            f"volta {self.lap - 1}: {self.lap_length:.0f} m, passou a "
            f"{self.closest:.2f} m da origem, correcao do GNSS "
            f"({self.offset[0]:+.1f}, {self.offset[1]:+.1f}) m")
        self.lap_length = 0.0
        self.closest = 1e9
        if self.laps and self.lap > self.laps:
            self.state = "done"
            self.stopped_reason = f"{self.laps} volta(s) completa(s)"

    def tick(self):
        dt = 1.0 / self.rate

        if self.state == "wait":
            if self.pos is None or self.fx is None:
                return
            if self.use_gnss and self.fix_origin is None:
                return              # still standing still, averaging fixes
            if self.cam_height is None and not self._lookup_camera():
                return
            self.state = "follow"
            # The clock on "how long since the line was last seen" starts
            # here and not at construction: everything before this was
            # waiting for the bridge, and none of it was blind driving.
            self.seen_at = time.monotonic()
            self.get_logger().info("na estrada, seguindo")

        if self.state == "done":
            self.out_speed = max(0.0, self.out_speed - self.accel * dt)
            self.commanded = 0.0
            self._publish()
            return

        if self.state == "follow":
            asked = self._steer_from_road()
            if asked is None:
                # Nothing to see. Hold the wheel where it is and ease off: on
                # an arc that is the right answer, and on a straight it costs
                # nothing. What it must not do is straighten up, which would
                # walk the machine off the outside of a bend.
                lost_for = time.monotonic() - self.seen_at
                target_speed = self.bend_speed if lost_for < 1.0 else self.turn_speed
                if lost_for > self.lost_timeout:
                    self.state = "done"
                    self.stopped_reason = f"estrada perdida por {lost_for:.1f} s"
            else:
                self.curvature = asked
                target_speed = (self.speed if abs(self.curvature) < 0.02
                                else self.bend_speed)

            if self.pos is not None and self._remaining() <= 0.0:
                action = self.route[self.leg][2]
                if action == "coast":
                    self._next_leg()
                else:
                    self.turn_radius = self.route[self.leg][3]
                    self.turn_sweep = self._sweep_to_next_leg()
                    self.turn_sign = math.copysign(1.0, self.turn_sweep)
                    self.turn_from = self.yaw
                    self.turned = 0.0
                    self.state = "turn"
            elif self.pos is not None:
                # Slow into a junction the way anything with a trailer does.
                if self._remaining() < 25.0:
                    target_speed = min(target_speed, self.turn_speed)

        if self.state == "turn":
            # Constant curvature, and the odometry says when 90 degrees of it
            # have gone by. The camera is not consulted: at a junction the
            # line it would lock onto is the one being left.
            step = self.yaw - self.turn_from
            self.turn_from = self.yaw
            self.turned += math.atan2(math.sin(step), math.cos(step))
            self.curvature = self.turn_sign / self.turn_radius
            target_speed = self.turn_speed
            if abs(self.turned) >= abs(self.turn_sweep):
                # The line this band was tracking is the one being left, so
                # both bands start again from the middle of the image.
                self.track = {"near": None, "far": None}
                self.near_y = self.far_y = None
                self.seen_at = time.monotonic()
                self.state = "follow"
                self._next_leg()

        # The wheel moves at a rate and the throttle at an acceleration,
        # because the Ackermann plugin is rate limited itself and stepped
        # commands only queue up.
        alpha = min(1.0, dt / self.tau)
        self.commanded += (self.curvature - self.commanded) * alpha
        if target_speed > self.out_speed:
            self.out_speed = min(target_speed, self.out_speed + self.accel * dt)
        else:
            self.out_speed = max(target_speed, self.out_speed - self.accel * dt)
        self._publish()

    def _lookup_camera(self):
        """How high the camera is, asked of the robot rather than assumed."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_footprint", "camera_front_link", rclpy.time.Time())
        except Exception:
            return False
        self.cam_height = tf.transform.translation.z
        self.get_logger().info(
            f"camera a {self.cam_height:.3f} m do solo; bandas em "
            f"{self.near_d:.0f} m (fileira {self._row_for(self.near_d)}) e "
            f"{self.far_d:.0f} m (fileira {self._row_for(self.far_d)})")
        return True

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.out_speed
        msg.angular.z = self.out_speed * self.commanded
        self.pub.publish(msg)

    def status(self):
        if self.pos is None:
            return "esperando odometria"
        radius = 1.0 / self.commanded if abs(self.commanded) > 1e-4 else float("inf")
        where = ("perdida" if self.near_y is None and self.far_y is None
                 else f"{self.near_y:+.2f} m" if self.near_y is not None
                 else f"({self.far_y:+.2f} m)")
        return (f"volta {self.lap} perna {self.leg + 1}/{len(self.route)} {self.state:6s} "
                f"({self.pos[0]:+7.1f},{self.pos[1]:+7.1f}) "
                f"faltam {max(0.0, self._remaining()):6.1f} m  "
                f"v {self.out_speed:4.1f} m/s  R {radius:8.1f} m  "
                f"eixo {where}{'  [cruzamento]' if self.wide else ''}")


def main():
    rclpy.init()
    node = LineFollow()
    period = 1.0 / node.rate
    last_print = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            node.tick()
            now = time.monotonic()
            if now - last_print > 0.5:
                last_print = now
                print("\r\033[K" + node.status(), end="", flush=True)
            if node.state == "done" and node.out_speed <= 0.0:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print()
        if node.stopped_reason:
            node.get_logger().info(node.stopped_reason)
        # Stop the machine on the way out, and only while there is still a
        # context to publish on: a Ctrl-C or a SIGTERM has already torn rclpy
        # down by the time this runs, and publishing into that raises rather
        # than stopping anything. What actually protects the tractor in that
        # case is the Ackermann plugin, which brakes on a stale command.
        if rclpy.ok():
            stop = Twist()
            for _ in range(5):
                node.pub.publish(stop)
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
