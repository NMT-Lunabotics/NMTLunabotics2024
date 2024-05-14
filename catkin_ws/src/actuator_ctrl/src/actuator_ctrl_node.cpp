#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_raw/CanFrame.h>
#include <vector>
#include <cmath>
#include <cstring> // Include for memcpy
#include "main_bus.hpp"

ros::Publisher can_pub;
ros::Subscriber joy_sub;
float actuator_max_vel;
int arm_axis, bucket_axis;
int prev_arm_vel = 0;   // Global variables to store previous velocities
int prev_bucket_vel = 0;

int velocity_deadzone(double axis_value) {
  if (axis_value < -0.5)
    return -5;
  else if (axis_value > 0.5)
    return 5;
  else
    return 0;
}

void callback(const sensor_msgs::Joy::ConstPtr& msg) {
    int arm = velocity_deadzone(msg->axes[arm_axis]);
    int bucket = velocity_deadzone(msg->axes[bucket_axis]);

    // Only send a CAN message if the velocity has changed
    if (arm != prev_arm_vel || bucket != prev_bucket_vel) {
        prev_arm_vel = arm;      // Update the global previous velocities
        prev_bucket_vel = bucket;

        can::ActuatorVelCommands cmd = {
          .arm_vel = (double)arm,
          .bucket_vel = (double)bucket,
        };
        uint8_t buffer[8];
        can::to_buffer(buffer, can::serialize(cmd));

        can_raw::CanFrame can_frame;
        can_frame.id = (short int) can::FrameID::ActuatorVelCommands;
        memcpy(&can_frame.data, buffer, sizeof(buffer));

        can_pub.publish(can_frame);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "actuator_ctrl_node");
    ros::NodeHandle nh;

    nh.getParam("actuator_max_vel", actuator_max_vel);
    std::string joy_topic;
    nh.getParam("joy_topic", joy_topic);
    nh.getParam("arm_axis", arm_axis);
    nh.getParam("bucket_axis", bucket_axis);

    joy_sub = nh.subscribe(joy_topic, 10, callback);
    can_pub = nh.advertise<can_raw::CanFrame>("/canbus", 10);

    ros::spin();
    return 0;
}
