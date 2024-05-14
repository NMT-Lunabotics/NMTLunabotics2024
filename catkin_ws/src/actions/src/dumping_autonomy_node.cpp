#include <actions/execute_dumping_autonomy.h>
#include <can_raw/CanFrame.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>

#include "main_bus.hpp"
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseActionResult.h>

// State of the overall program.
struct State {
  std::atomic<bool> dumping;
  std::atomic<bool> move_base_success; // New variable to store move base result
  std::atomic<int> arm_pos;
  std::atomic<int> bucket_pos;

  ros::NodeHandle nh;
  ros::ServiceServer service;
  ros::Subscriber can_subscriber;
  ros::Subscriber move_base_subscriber; // New subscriber for move base results
  ros::Publisher can_publisher;

  std::thread main_thread;

  State()
      : move_base_success(false) // Initialize move_base_success
  {
    std::cout << "Dumping autonomy initialized\n";

    // we get to manually initialize things here??
    dumping = false;
    arm_pos = 0;
    bucket_pos = 0;

    service = nh.advertiseService("dump", &State::handle_service, this);
    can_subscriber =
        nh.subscribe("/canbus_input", 16, &State::handle_can_msg, this);
    move_base_subscriber =
        nh.subscribe("/move_base/result", 10, &State::handle_move_base_result,
                     this); // Listen to move base results
    can_publisher = nh.advertise<can_raw::CanFrame>("/canbus", 16);

    // Spawn the thread.
    main_thread = std::thread(&State::thread_main, this);
  }

  void send_actuator_commands(double arm_vel, double bucket_vel) {
    can::ActuatorVelCommands cmds = {
        .arm_vel = arm_vel,
        .bucket_vel = bucket_vel,
    };

    can_raw::CanFrame frame;
    frame.id = (int)can::FrameID::ActuatorVelCommands;
    can::to_buffer(frame.data.data(), can::serialize(cmds));
    can_publisher.publish(frame);
  }

  void send_drive_commands(double left_vel, double right_vel) {
    can::MotorCommands cmds = {
        .left = {.speed = left_vel},
        .right = {.speed = right_vel},
    };

    can_raw::CanFrame frame;
    frame.id = (int)can::FrameID::MotorCommands;
    can::to_buffer(frame.data.data(), can::serialize(cmds));
    can_publisher.publish(frame);
  }

  void thread_main() {
    while (true) {
      usleep(0.5e6); // Wait for a half-second before checking again
      if (!dumping || !move_base_success) // Only proceed if dumping is
                                          // triggered and move base succeeded
      {
        continue;
      }

      std::cout << "Move base success, starting dump sequence.\n";

      // Dump sequence as previously defined
      // Extend the arms and bucket, then jiggle, drive backwards, and retract
      // to initial positions

      // Reset move base success state
      move_base_success = false;
      dumping = false; // Reset dumping state at the end
    }
  }

  bool handle_service(actions::execute_dumping_autonomyRequest &req,
                      actions::execute_dumping_autonomyResponse &res) {
    dumping = true; // Trigger dumping from the service request
    return true;
  }

  void handle_move_base_result(
      const move_base_msgs::MoveBaseActionResult::ConstPtr &result) {
    if (result->status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
      move_base_success = true;
    }
  }

  void handle_can_msg(const can_raw::CanFrame &frame) {
    if (frame.id == 4) {
      // ActuatorArmPos
      int left_pos = frame.data[0];
      int right_pos = frame.data[1];
      arm_pos = (left_pos + right_pos) / 2;
    } else if (frame.id == 5) {
      // ActuatorBucketPos
      bucket_pos = frame.data[0];
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dumping_autonomy");
  State state;
  ros::spin();
}
