# NMT Lunabotics 2024

This repository contains the codebase for NMT Lunabotics' 2024 robot,
which is named BOWIE.

## Robot hardware

The robot itself has the following sensors:
- Two Intel D455 depth cameras, used for collecting point clouds and
  assembling into a map. One on the front left, and the other on the
  front right; the cameras are mounted on small metal columns to avoid
  them moving up and down with the arms, which would have required
  precise calculations to account for in the point cloud processing.
- One Intel T265 tracking camera, used for localization.
- One Arducam USB camera mounted on the front of the robot directly
  above the bucket, for manual driving, and one (the "Funkencam")
  mounted near the back of the robot looking forward, for observing
  the dump routine and determining whether the bucket was empty.
- One Intel D435 depth camera mounted on the back of the robot
  pointing backwards, used for locating a single April tag for
  orienting the robot within the arena.

These are used to assist in directing the following actuators:
- Two Midwest Motion motors, one for the left side of the robot and
  the other for the right side. The two left wheels are kept in sync
  using a chain inside a custom-machined metal drive train structure,
  and the same is true of the right-side wheels. These are controlled
  using two motor drivers that take two digital inputs from 0 to 10
  volts and produce torque proportional to the difference between the
  two inputs, which allows us to easily make the motors spin
  backwards.
- Three Midwest Motion linear actuators: one for the left side of the
  arm, another for the right side of the arm, and a third (shorter
  one) for turning the bucket up and down. These have five pins each:
  two for controlling the direction and velocity of the actuators as
  with the motors, and three connected to a potentiometer that can be
  used to measure with relatively high precision the position of the
  actuators; this is important because we need the arm actuators to
  always be as close as possible in length to avoid bending the
  robot's metal.

Each of the cameras is connected to an NVIDIA Jetson Orin AGX (the
main computer) via its USB ports, but the motors and actuators are
connected via a much more cumbersome scheme. The electronics box
contains three Arduino Uno R4 WiFi boards, the first of which controls
the two motors, the second of which controls the bucket actuator, and
the third of which controls the two arm actuators. We use a CAN bus
connected to the Jetson's CAN0 output to communicate commands and
motor feedback with them.

The electronics box also contains three MD04 H-bridges to send
commands to the linear actuators, two buck converters to get from 48
volts to 24 and 12 volts to power individual components, and two PWM
to analog converters to assist in driving the motors from the
Arudinos.

## Strategy

The design of this robot sought out autonomous navigation, digging,
and dumping; all three are functional at some level. Digging and
dumping are simple pre-recorded actions, but navigation requires a
more complex strategy. The autonomous navigation system begins
building a map as soon as the back camera catches sight of the April
tag, as this allows us to know where in the arena we are; the April
tag is then ignored for the rest of the run, and we _only_ use the
T265's odometry to track the robot's movements in the future. We build
the map using point clouds from the two D455s on the front of the
robot: first, we merge them and convert the result into a heightmap,
and then we use the derivative of that heightmap to calculate slopes,
and we use a simple threshold of slope to compute traversability. The
robot then uses a simple path planner to find routes from its current
position to known waypoints in the arena, then executes the
pre-recorded actions; this mode of operation theoretically allows full
autonomy, but it was never achieved.
