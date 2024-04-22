#include "main_bus.hpp"
#include <can_convert/ArmStatus.h>
#include <can_raw/CanFrame.h>
#include <cmath>
#include <ros/ros.h>
#include <vector>

ros::Subscriber can_sub;
ros::Publisher status_pub;

float arm_angle;
float bucket_angle;

double bucketAngle(double dist) {
  double dist_in = 0.03937008 * dist;
  double phi = 0;
  double theta = 0;
  double gamma = 0;
  double angle = 0;
  double s1 = 17.852;
  double s0 = 12.6875;
  double s2S = 15.164;
  double s2 = dist_in + s2S;
  double s3 = 25.080;
  double s4 = 16.164;
  gamma = acos((s3 * s3 + s1 * s1 - s2S * s2S) / (2 * s3 * s1));
  phi = acos((s3 * s3 + s1 * s1 - s2 * s2) / (2 * s3 * s1));
  angle = phi - gamma;
  return angle;
}

int armAngle(double dist) { return 0; }

void callback(const can_raw::CanFrame::ConstPtr &msg) {
  if (msg->id == (int)can::FrameID::ActuatorArmPos) {
    can::ActuatorArmPos arm_pos =
        can::ActuatorArmPos_deserialize(can::from_buffer(msg->data.data()));
    // TODO maybe check if left and right are different?
    arm_angle = armAngle(arm_pos.left_pos);
  }
  if (msg->id == (int)can::FrameID::ActuatorBucketPos) {
    can::ActuatorBucketPos bucket_pos =
        can::ActuatorBucketPos_deserialize(can::from_buffer(msg->data.data()));
    // TODO maybe check if left and right are different?
    bucket_angle = bucketAngle(bucket_pos.pos);
  }
  can_convert::ArmStatus status;
  status.arm_angle = arm_angle;
  status.bucket_angle = bucket_angle;
  std::cout << bucket_angle << "\n";
  status_pub.publish(status);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_convert_node");
  ros::NodeHandle nh;

  can_sub = nh.subscribe("canbus_input", 10, callback);

  status_pub = nh.advertise<can_convert::ArmStatus>("/arm_status", 10);

  ros::spin();
  return 0;
}
