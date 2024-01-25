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
  ];

  programs.ros.myIP = "192.168.0.235";
  services.ros.rosbridge.enable = true;

  services.ros.elevationMapping.build = true;

  services.ros.realsense2.enable = false;

  services.ros.runServices.canRawNode = {
    packageName = "can_raw";
    executable = "can_raw_node";
    workspace = "/home/lunabotics/goliath/catkin_ws";
  };
}

