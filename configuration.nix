# This file is for ros4nix.

{ config, pkgs, ... }:
{
  # ---- Base NixOS configuration ----
  imports = [
    ./ros
  ];

  system.stateVersion = "23.05";
  boot.isContainer = true;

  # Change this to the architecture of your system.
  nixpkgs.hostPlatform = "aarch64-linux";

  # ---- Begin ROS configuration ----

  services.ros.enable = true;

  programs.ros.packages = [
    "xacro"
    "realsense2-camera"
    "robot-state-publisher"
    "teleop-twist-joy"
    "joy"
    "teleop-twist-keyboard"
    "teb-local-planner"
    "navigation"
    "tf2-tools"
    "usb-cam"
    "image-transport"
    "theora-image-transport"
    "compressed-image-transport"
  ];

  programs.ros.ubuntuPackages = [
    "libeigen3-dev"
  ];
  programs.ros.myIP = "192.168.0.103";
  services.ros.rosbridge.enable = true;

  services.ros.elevationMapping.build = true;

  services.ros.realsense2.enable = false;

  # services.ros.runServices.canRawNode = {
  #   packageName = "can_raw";
  #   executable = "can_raw_node";
  #   workspace = "/home/lunabotics/goliath/catkin_ws";
  # };
  # currently included in motor_ctrl/motor.launch

  # Turn on the `motion` LED as soon as Linux boots and gets to this
  # point.
  systemd.services.ros-motion-led = {
    wantedBy = [ "multi-user.target" ];
    after = [ "rosMaster.service" ];
    script = ''
      /var/ros/nixWrappers/rostopic pub /leds/motion std_msgs/Bool true
    '';
  };

  services.ros.runServices.usb-cam = {
    packageName = "usb_cam";
    executable = "usb_cam_node";
  };

  services.ros.runServices.heartbeat_client = {
    workspace = "/home/lunabotics/goliath/catkin_ws";
    packageName = "heartbeat";
    executable = "heartbeat_client_node";
  };

  services.ros.runServices.leds = {
    workspace = "/home/lunabotics/goliath/catkin_ws";
    packageName = "leds";
    executable = "leds_node";
  };

  services.ros.launchServices.motor-ctrl = {
    packageName = "motor_ctrl";
    launchFile = "motor.launch";
    workspace = "/home/lunabotics/goliath/catkin_ws";
  };

  services.ros.launchServices.actuator-ctrl = {
    packageName = "actuator_ctrl";
    launchFile = "actuator.launch";
    workspace = "/home/lunabotics/goliath/catkin_ws";
  };

  services.ros.launchServices.test-map = {
    packageName = "mapping";
    launchFile = "test_map.launch";
    workspace = "/home/lunabotics/goliath/catkin_ws";
  };
}

