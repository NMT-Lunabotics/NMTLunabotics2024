#include <ros/ros.h>
#include <can_raw/CanFrame.h>
#include <can_convert/ArmStatus.h>
#include <vector>
#include <cmath>
#include "main_bus.hpp"

ros::Subscriber can_sub;
ros::Publisher status_pub;

void callback(const can_raw::CanFrame::ConstPtr& msg) {
    can_convert::ArmStatus status;
    status.arm_angle = 30;
    status.bucket_angle = 30;
    status_pub.publish(status);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_convert_node");
    ros::NodeHandle nh;

    can_sub = nh.subscribe("canbus", 10, callback);

    status_pub = nh.advertise<can_convert::ArmStatus>("/arm_status", 10);

    ros::spin();
    return 0;
}

