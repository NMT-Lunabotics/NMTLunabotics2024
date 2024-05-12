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

  # # Have the ROS master force all the motors to stop when its service
  # # is stopped.
  # systemd.services.rosMaster.serviceConfig.ExecStopPost =
  #   pkgs.writeScript "stop-robot"
  #     ''
  #       #!/bin/sh
  #       cansend can0 001#8000800000000000 || true
  #       cansend can0 003#8080000000000000 || true
  #       cansend can0 007#ffffffffffffffff || true # Turn on all lights
  #     '';

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
    "apriltag-ros"
    "cv-bridge"
  ];

  programs.ros.ubuntuPackages = [
    "libeigen3-dev"
  ];

  programs.ros.defaultWorkspace = "/home/lunabotics/goliath/catkin_ws";
  programs.ros.myIP = "192.168.5.11";
  # programs.ros.myIP = "192.168.0.207";
  services.ros.rosbridge.enable = true;

  services.ros.elevationMapping.build = true;

  programs.ros.buildPackages =
    let
      any_node = pkgs.fetchFromGitHub {
        owner = "leggedrobotics";
        repo = "any_node";
        rev = "0.5.0";
        sha256 = "sha256-N9yDfoaF1EAmS1gH7G0YfOpBIzMApgHhYF7OSc7Cbgo=";
      };
    in
    {
      traversability_estimation = pkgs.fetchFromGitHub {
        owner = "leggedrobotics";
        repo = "traversability_estimation";
        rev = "SRC_Final";
        sha256 = "sha256-Cb37ar7XnVWkOgncjF5ZctwWpHJ38DTSAVQYUDaL2gQ=";
      };

      param_io = "${any_node}/param_io";
      any_node = "${any_node}/any_node";
      any_worker = "${any_node}/any_worker";
      signal_handler = "${any_node}/signal_handler";
    };

  services.ros.realsense2.enable = false;

  services.ros.launchServices = {
    motor-ctrl = {
      packageName = "motor_ctrl";
      launchFile = "motor.launch";
      workspace = "/home/lunabotics/goliath/catkin_ws";
    };

    l515 = {
      packageName = "realsense2_camera";
      launchFile = "rs_camera.launch";
      args = {
        camera = "l515";
        device_type = "l515";
        filters = "pointcloud";
        depth_fps = "30";
        depth_width = "640";
        depth_height = "480";
        pointcloud_texture_stream = "RS2_STREAM_ANY";
      };
    };

    t265 = {
      packageName = "realsense2_camera";
      launchFile = "rs_t265.launch";
      args = {
        camera="t265";
      };
    };

    mapping = {
      packageName = "mapping";
      launchFile = "mapping.launch";
      workspace = "/home/lunabotics/goliath/catkin_ws";
    };

    move-base = {
      packageName = "control";
      launchFile = "move_base.launch";
      workspace = "/home/lunabotics/goliath/catkin_ws";
    };
  };

  services.ros.staticTransforms =
    let
      inch = inches: inches * (25.4 / 1000);
    in
    [
      {
        parent = "map";
        child = "t265_odom_frame";
      }

      {
        parent = "t265_link";
        child = "base_link";
        x = inch (-3);
        y = inch (0);
        z = inch (-4);
      }

      {
        parent = "t265_link";
        child = "l515_link";
        x = inch (1);
        y = inch (0);
        z = inch (3.5);
	pitch = 15;
      }
    ];
}
