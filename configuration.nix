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
  # programs.ros.myIP = "192.168.5.5";
  # programs.ros.myIP = "192.168.0.213";
  programs.ros.myIP = "192.168.0.207";
  # programs.ros.myIP = "192.168.124.35";
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
      /var/ros/nixWrappers/rostopic pub --once /leds/motion std_msgs/Bool true
    '';
  };

  # DHCP insurance. It's stupid.
  systemd.services.ros-dhcp-insurance = {
    wantedBy = [ "multi-user.target" ];
    script = ''
      sleep 1
      while true; do
          /usr/sbin/ifconfig eth0 192.168.5.5
          /usr/bin/systemctl start kea-dhcp4-server
          sleep 1
      done
    '';
  };

  services.ros.runServices = {
    usb-cam-arducam = {
      packageName = "usb_cam";
      executable = "usb_cam_node";
      namespace = "arm_cam";
      remap._video_device =
        "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_USB_Camera_UC684-video-index0";
      rosParams = pkgs.writeText "params" ''
        framerate: 10
      '';
    };

    usb-cam-logitech = {
      packageName = "usb_cam";
      executable = "usb_cam_node";
      namespace = "funk_cam";
      remap._video_device =
        "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_Live_Camera_SN0001-video-index0";
      rosParams = pkgs.writeText "params" ''
        framerate: 10
      '';
    };

    heartbeat_client = {
      workspace = "/home/lunabotics/goliath/catkin_ws";
      packageName = "heartbeat";
      executable = "heartbeat_client_node";
    };

    leds = {
      workspace = "/home/lunabotics/goliath/catkin_ws";
      packageName = "leds";
      executable = "leds_node";
    };

    digging-autonomy = {
      workspace = "/home/lunabotics/goliath/catkin_ws";
      packageName = "actions";
      executable = "digging_autonomy";
    };

    dumping-autonomy = {
      workspace = "/home/lunabotics/goliath/catkin_ws";
      packageName = "actions";
      executable = "dumping_autonomy";
    };

    limiter-left = {
      workspace = "/home/lunabotics/goliath/catkin_ws";
      packageName = "limiter";
      executable = "pointcloud_limiter";
      rawArgs = [ "left" "/d455_left/depth/color/points_cropped" ];
    };
    limiter-right = {
      workspace = "/home/lunabotics/goliath/catkin_ws";
      packageName = "limiter";
      executable = "pointcloud_limiter";
      rawArgs = [ "right" "/d455_right/depth/color/points_cropped" ];
    };
  };

  services.ros.launchServices = {
    motor-ctrl = {
      packageName = "motor_ctrl";
      launchFile = "motor.launch";
      workspace = "/home/lunabotics/goliath/catkin_ws";
    };

    actuator-ctrl = {
      packageName = "actuator_ctrl";
      launchFile = "actuator.launch";
      workspace = "/home/lunabotics/goliath/catkin_ws";
    };

    hud = {
      packageName = "hud";
      launchFile = "hud.launch";
      workspace = "/home/lunabotics/goliath/catkin_ws";
    };

    # continuous_detection = {
    #   packageName = "mapping";
    #   launchFile = "continuous_detection.launch";
    #   workspace = "/home/lunabotics/goliath/catkin_ws";
    # };

    # camera-right = {
    #   packageName = "realsense2_camera";
    #   launchFile = "rs_camera.launch";
    #   args = {
    #     camera = "d455_right";
    #     device_type = "d455";
    #     serial_no = "213522250920";
    #     filters = "pointcloud";
    #     depth_fps = "30";
    #     depth_width = "640";
    #     depth_height = "480";
    #     enable_color = "true";
    #     pointcloud_texture_stream = "RS2_STREAM_ANY";
    #   };
    # };

    # camera-left = {
    #   packageName = "realsense2_camera";
    #   launchFile = "rs_camera.launch";
    #   args = {
    #     camera = "d455_left";
    #     device_type = "d455";
    #     serial_no = "213522253528";
    #     filters = "pointcloud";
    #     depth_fps = "30";
    #     depth_width = "640";
    #     depth_height = "480";
    #     enable_color = "true";
    #     pointcloud_texture_stream = "RS2_STREAM_ANY";
    #   };
    # };

    # camera-d435 = {
    #   packageName = "realsense2_camera";
    #   launchFile = "rs_camera.launch";
    #   args = {
    #     camera = "d435";
    #     serial_no = "102122072092";
    #     enable_color = "true";
    #     pointcloud_texture_stream = "RS2_STREAM_ANY";
    #   };
    # };

    cameras = {
      packageName = "mapping";
      launchFile = "cameras.launch";
      workspace = "/home/lunabotics/goliath/catkin_ws";
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
        x = inch (10.53);
        y = inch (-10.273);
        z = inch (14.337);
        # pitch = -90;
      }

      {
        parent = "t265_link";
        child = "base_link_real";
        x = inch (-10.53);
        y = inch (-10.273);
        z = inch (-10.0);
        yaw = 90;
      }

      {
        parent = "base_link_real";
        child = "base_link";
        z = inch (-36);
      }

      {
        parent = "base_link_real";
        child = "d455_left_lr";
        x = inch (10.579);
        y = inch (10.841);
        z = inch (18.034);
        yaw = 30;
      }

      {
        parent = "d455_left_lr";
        child = "d455_left_ud";
        y = inch (0.715);
        z = inch (1.569);
        pitch = 21;
      }

      {
        parent = "d455_left_ud";
        child = "d455_left_link";
        x = inch (0.893);
        y = inch (0.586);
        z = inch (-0.550);
      }

      {
        parent = "base_link_real";
        child = "d455_right_lr";
        x = inch (10.579);
        y = inch (-10.841);
        z = inch (18.034);
        yaw = -30;
      }

      {
        parent = "d455_right_lr";
        child = "d455_right_ud";
        y = inch (-0.715);
        z = inch (1.569);
        pitch = 20;
      }

      {
        parent = "d455_right_ud";
        child = "d455_right_link";
        x = inch (0.893);
        y = inch (-0.586);
        z = inch (1.550);
      }

      {
        parent = "base_link_real";
        child = "d435_link";
        # measurements from Niall's phone
        x = inch (-23);
        y = inch (8);
        z = inch (17.5);
        yaw = 180;
      }

      {
        parent = "tag";
        child = "map";
        # y = -1.25;
        y = -(
          2 - inch (8.5)
        );
        # z = inch (-7);
        z = inch (-14.5);
        x = -0.17;              # Pipe width
        # roll = 90;
        # pitch = 90;
        # yaw = 180;
      }

      {
        parent = "tag_0";
        child = "tag_righted";
        roll = -90;
        pitch = -90;
      }

      # ---- Arena waypoints ----
      {
        parent = "map";
        child = "robot_start";
        x = 1;
        y = 1;
      }

      {
        parent = "map";
        child = "excavation_center";
        x = 3.88 + 1.5;
        y = 2 + 1.5;
      }

      {
        parent = "map";
        child = "dump_center";
        x = 3.88 + 1.5;
        y = 1.5;
        yaw = -90;
      }
    ];
}
