# NMT Lunabotics 2024

![./bowie.png](BOWIE the robot in the NMT Lunabotics arena.)

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

## Deployment architecture

The main software is built on ROS Noetic, but since that's a pain to
install on machines other than the specific ones that ROS targets, and
because we wanted to avoid using Docker after our experiences in 2023,
we use a custom Nix-based deployment at
https://github.com/AlexBethel/ros4nix; it is designed to be easy to
install on any Linux that can support Nix (the vast majority of them)
and provide a relatively seamless way to use ROS and install packages
from the official repositories and from source, and to schedule ROS
node and masters to start at boot time. `ros4nix` was originally
designed to replace `roslaunch` and `launch` files _entirely_, but for
a number of ergonomics reasons we still use some `launch` files. Due
to a bit of an architecture disagreement all the cameras are coded in
as both `ros4nix` services and `launch` files, but only the `launch`
files are currently used by default.

The Nix configuration is in `configuration.nix` in this directory, and
all the ROS source code is in `catkin_ws`.

## Catkin packages

We used our own different Catkin packages for different tasks.

### `actions`

Pre-recorded actions like dumping, digging, and full autonomy. There
are four nodes: `dumping_autonomy`, `digging_autonomy`, `fuckit`, and
`funkenstein`. `dumping_autonomy` and `digging_autonomy` run in the
background and listen on the `/dump` and `/dig` services for requests
to dump and dig, respectively; `fuckit` runs the (never successfully
tested) full autonomy; and `funkenstein` was originally going to be a
simple programming language specifically for writing autonomy scripts,
but was never fleshed out.

Note that `/dig` immediately starts digging, but `/dump` waits for a
`move_base` goal completion before it starts dumping, as competition
scoring counts autonomous navigation to the dump zone as part of the
autonomous dumping category.

### `actuator_ctrl`

A single node that subscribes to the `joy_teleop/joy` node and
publishes commands to the `/canbus` CAN bus proxy.

### `can_convert`

A node that listens to the CAN bus feedback (`/canbus_input`) and
does the necessary trigonometry to calculate the current angles of the
bucket and arm.

### `can_raw`

A node listens to the `/canbus` topic and sends them using Linux's CAN
drivers to the `can0` output. This allows any ROS node to direct the
motors, actuators, etc.

### `can_raw_input`

The opposite of `can_raw`: listens to `can0` and publishes CAN frames
to the ROS topic `/canbus_input`.

### `control`

Mainly contains launch files: `teleop.launch`, which opens camera
feeds and uses a local controller joystick to control the robot's
motors and actuators, and also `move_base.launch`, which starts
`move_base` with some preset parameters. There is also source code for
a `dump_node` here where pushing a button on the controller would make
it enter the dumping cycle, but it never got used.

### `heartbeat`

A "server" and "client" node. The server sends heartbeat messages to
the client, and when the client (the robot) receives them, it
publishes a message to `/leds` to turn on the blue LED on the robot
indicating that it is actively being controlled and people should stay
away. This safety feature was only ever implemented for manual
control, as the autonomous control subsystems were always able to move
even while the blue light was off, which kind of defeats the purpose.

### `hud`

Overlays the camera feeds with an abstract sketch of the robot's arm
positions, to assist in manual driving.

### `leds`

Listens on three ROS topics: `/leds/error`, `/leds/autonomy`, and
`/leds/motion`, for `std_msgs/Bool` messages. These allow nodes to
turn on and off the three LEDs on the robot as necessary; however, the
`error`, `autonomy`, and `motion` LEDs ended up being used for
different purposes and do not, in fact, indicate error, autonomy, and
motion respectively.

### `limiter`

A generic speed throttler for ROS topics, with specific instantiations
for grid maps and point clouds. ROS's C++ binding (as far as I am
aware) makes it impossible to write these nodes in a truly generic
way, so the main code is in `limiter.hpp` and the instantiations are
extremely simple files that include that header file for the real
work. The use of this node (along with a couple other efforts) cut our
bandwidth usage by a factor of over 100 between our first KSC run and
our second.

### `mapping`

Configuration and code to get the mapping system working. This package
contains `mapping.launch` which runs `elevation_mapping` to build and
process the height map, and a node called `cvt_occupancy` to convert
that height map into an occupancy grid for direct navigation.
`mapping.launch` also supports an option called `plan_b` that ignores
all mapping data and assumes the entire universe is traversable.

This package also contains `cameras.launch`, the non-`ros4nix` way of
running the cameras which we used for all competition runs.

We also have `continuous_detection.launch` and its associated program,
`april_tag_loc`, which searches for the April tag and publishes its
transform as soon as it is found.

There is a URDF file here which is *not used;* we use static transform
publishers specified in `configuration.nix` instead.
`transform.launch` is part of this URDF support and is also unused.

### `motor_ctrl`

Listens for joystick commands to send CAN commands to the motors.

### `move_to`

A simple node where you publish a string like `excavation_zone` and it
sends a `move_base` navigation goal to move the robot to the center of
the TF frame named `excvation_zone`. This node is used extensively by
the full autonomy system, `fuckit`.

## CAN Schema

We moved from using KCD files for specifying CAN messages to
generating them using our own software. We wrote a YAML file
containing CAN message descriptions and used
https://github.com/NMT-Lunabotics/canspec to generate a KCD file for
monitoring, as well as C++ headers that were much higher-quality than
`cantools`'s standard generated code. The generated header file is
symlinked into a bunch of the Arduino projects and the Catkin
projects, because making them actually depend on it as a package
properly would be hard and not worth it because Arduino and Catkin use
completely different build systems.
