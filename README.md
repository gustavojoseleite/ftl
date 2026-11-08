# ftl

**Follow the leader**: a line of agricultural machines where the first is driven and the others
follow it. This repository is the project's own code, the part no other project uses. The
platform it runs on (vehicles, worlds, driving) comes from `robot_description`, `agri_gazebo` and
`vehicle_control`, listed with this repository in the workspace's `projects/ftl/ftl.repos`.

```
ftl_follower/     pose_estimator (every vehicle) and follower (each follower)
road_follow/      drives a vehicle down the carreadores by its camera — FTL's leader driver
ftl_bringup/      the FTL nodes and their parameters, as one launch. No simulator.
├── launch/ftl.launch.py
└── config/ftl.yaml           one file per experiment is the intended use
ftl_gz/           FTL test scenarios in Gazebo: world, leader, followers, where they start
├── launch/sim.launch.py      the scenario only (world + convoy)
└── launch/ftl_sim.launch.py  the scenario and the FTL nodes, in one command
```

Leader perception (`ftl_perception`) and messages of its own (`ftl_msgs`) join as packages here
when they are written. A package moves out to a repository
of its own only if a second project needs it.

## The rule that keeps it portable

Only `ftl_gz` knows a simulator exists. `ftl_bringup` and every node talk to a vehicle through
the topics in its namespace — `cmd_vel`, `odom`, `gnss/fix`, `imu/data`, `camera_front/*` — and
its TF, so the same launch drives the convoy in Gazebo or on the farm; `use_sim_time` is the only
switch. Dependencies point one way: `ftl_gz` → `ftl_bringup`, `ftl_gz` → `agri_gz_bringup`, never
back.

Each vehicle keeps its TF tree in its own namespace (`/pickup/tf`, `/pickup/tf_static`), so a node
that reads TF takes the remaps `/tf → tf` and `/tf_static → tf_static`. `ftl.launch.py` does.

## Running a test

From the workspace root (the harness), in containers:

```bash
scripts/up.sh ftl                          # Gazebo + the FTL nodes, one container each
GUI=false FOLLOWERS=4 scripts/up.sh ftl -d
docker compose -p ftl restart ftl          # new FTL code, Gazebo keeps running
docker compose -p ftl down
```

Or by hand, inside `./docker/run.sh`:

```bash
ros2 launch ftl_gz ftl_sim.launch.py                       # both at once
ros2 launch ftl_gz sim.launch.py followers:=4 gui:=false   # the scenario alone…
ros2 launch ftl_bringup ftl.launch.py use_sim_time:=true   # …and the nodes, in a second shell
ros2 launch ftl_bringup ftl.launch.py params_file:=/workspace/exp_01.yaml
```

The default scenario: a pickup leading two autonomous tractors down the main carreador of
`citrus_farm.sdf`, ten metres apart; `road_follow` drives the pickup round the lap and each
tractor follows the machine in front of it. Scenario arguments: `world`, `leader`, `follower`,
`followers`, `spacing`, `yaw`, `gui`.

## How a follower follows

```
every vehicle    pose_estimator   odom + imu/data + gnss/fix  ->  ftl/state   (shared frame ftl_map)
each follower    follower         own ftl/state + ftl/state of the vehicle ahead  ->  cmd_vel
```

**One frame for the whole convoy.** Each vehicle's odometry starts where that vehicle was
spawned, so two odometries cannot be compared. `pose_estimator` puts every machine in the same
east/north frame, metres from one datum (`datum_lat`/`datum_lon` in `ftl.yaml`, the origin of the
citrus-farm worlds): heading from the IMU, which reports orientation in the world's ENU frame;
position dead-reckoned from odometry along that heading and pulled towards the GNSS fix with a
four-second time constant, the antenna's offset from `base_footprint` taken off first. Its output,
`ftl/state` (`nav_msgs/Odometry`), is what would travel over the radio between real machines.

**Path, not position.** `follower` keeps the positions of the vehicle ahead as a trail and steers
by pure pursuit on the trail point one lookahead (4 m + 1 s × speed, at most 10 m) ahead of
itself. Aiming at the vehicle itself would cut every corner of a carreador.

**Gap along the path.** How far behind is the arc length of that trail between the two machines,
so a bend does not read as being too close. Speed is the vehicle ahead's speed plus 0.4 (m/s)/m of
gap error, ramped at 1 m/s², capped at 4 m/s, and zero below `min_gap` (5 m) or when either
`ftl/state` is more than a second old. Each follower follows the one in front of it, so
`tractor_2` follows `tractor_1`, not the pickup.

Every number above is a parameter in `ftl_bringup/config/ftl.yaml`. For experiments, each follower
also publishes `ftl/gap` (metres along the path) and `ftl/cross_track` (metres off it, left
positive).

**Measured** (2026-11-08, `citrus_farm.sdf`, pickup + 2 tractors, `road_follow` lap at 3 m/s, past
the first fifth of each run):

| | 222 m straight | 358 m with the 90° turn at the east boundary |
|---|---|---|
| gap, set to 10 m | 9.47 – 10.56 m, mean 10.02–10.03 | 8.30 – 10.83 m, mean 9.99–10.04 |
| off the leader's path | mean 0.12–0.13 m, p95 0.30 m, max 0.42 m | mean 0.14 m, p95 0.36 m, max 0.66 m |
| estimate vs Gazebo's true pose, at the end | 0.14 – 0.41 m | — |

Unit tests for the trail, the control laws and the geodesy: `python3 -m pytest
src/ftl/ftl_follower/test` (16 tests).

The followers carry GNSS and IMU but no camera (`followers_camera:=false` in the convoy), so the
convoy costs one camera however long it is.

## road_follow: driving by the edges of the road

```bash
ros2 run road_follow road_follow                      # laps until stopped
ros2 run road_follow road_follow --ros-args -p laps:=1 -p speed:=2.0
ros2 run road_follow road_follow --ros-args -p route:=follow   # no route: just keep to the road
```

The tractor leaves the crossroads at the centre of the world heading east, takes the main
carreador out to the east boundary, turns left, runs 244 m up the boundary and round its 24 m
corner, turns left again onto the carreador that comes back down the middle of the property, and
arrives at the crossroads it started from. **1109 m, a quarter of the network**, and then it does
it again, coming back within **1.8 m** of the world origin each time — a corner taken close rather
than a dot hit, because an Ackermann machine cannot arrive at a point and leave it in another
direction.

**The steering is the camera and nothing else.** What it follows is the **running surface**: a
carreador is 12 m of bare earth with 7 m of graded track down the middle, and that inner strip is
a few counts lighter than the shoulders either side, so its two edges sit at ±3.5 m and their
midpoint is the centreline. A pair of edges is better than one mark — the midpoint is the middle
of the road whether or not the machine is square to it, and a junction, where the joining road's
surface widens the strip past 7 m, fails the spacing test and is rejected rather than followed.
Where one edge is lost the other still fixes the centre 3.5 m to its side, on a tighter gate.

Nothing here is a threshold that depends on the weather. The step from surface to shoulder is
**five to eight counts** of red, and the surface itself changes with the light and the slope — 176
counts 8 m ahead, 156 at 14 m — so the row is smoothed over five pixels and compared against its
**own median** across a 10 m window: the surface is most of that window by construction, so the
median is the surface. The two bands look 6 m and 14 m ahead, and which pixel row that is comes
from `/camera_front/camera_info` and the camera's height in `tf`, so a camera moved in the
description moves the bands with it. Pure pursuit on the far band, a small pull on the near one,
and out goes `angular.z = speed × curvature`.

`route:=` picks which of them: `lap` is the circuit above, `follow` is no route at all — no
junctions, no turns, the machine simply keeps to whatever carreador it is standing on until it is
stopped, which is what a road with one way out wants — and `origin_bend` runs 43 m east from the
crossroads and bends onto the 30° road. The last two are what the films in
`docs/` of the workspace repository were shot on.

**How far a turn goes comes from the route's own directions**, not from a number written beside
it. Every turn used to run until the machine had swung 90°, which is right for three sides of a
square and was never noticed until a 30° turn was asked for: the tractor turned 90°, left the
carreador and drove into the talhão at a confident 3 m/s. The sweep is now the angle between the
leg being left and the leg being joined, so it cannot disagree with the route it belongs to.

What odometry and the GNSS are for is the one question the road cannot answer: at a junction where
two carreadores cross, which of them is mine. That is a map question, not a perception one. So the
route carries the three turns and the machine's own position says when one has arrived — and
**position here is the receiver on the mast, smoothed by odometry**, not odometry alone. Odometry
alone was tried: one lap of this circuit came back **9 m out**, which is ordinary for a slipping
Ackermann machine and quite enough to start a turn in the wrong place by the third lap. The fix
does not drift, so the two are fused the obvious way round — odometry for smoothness between
fixes, the fix for where that smoothness is anchored, four seconds of time constant. The
correction it applies is reported at the end of every lap, and it is not small: **−22.5 m** of x
on the lap measured here, all of it odometry error the receiver took back out. Without it this
circuit does not survive its third lap.

**Two marks were tried down the middle of the road before this, and both came out.** First a
30 cm white centreline, added as a reference: the follower tracked it beautifully until it did
not — twice it turned down the 30° road at x = 48 and drove into the talhão, because a joining
road's surface was laid over the line it joined and buried fourteen metres of it (see the z-order
note below, which is fixed). Then the line came out of the worlds for the reason it never should
have gone in: nobody paints a farm track. Then the wheel ruts every carreador wore, which are at
least a thing a dirt road has — except that as two hard-edged stripes 2.1 m apart they read as
paint as well, and matched the bitola of no machine on the property. They came out too.

What is left is the road itself, and it is the better target. Nothing on a carreador here is drawn
that a graded road would not have.
