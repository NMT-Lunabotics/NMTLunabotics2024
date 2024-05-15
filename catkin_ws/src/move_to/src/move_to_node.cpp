// Node for moving to a particular tf transform.

#include <ostream>
#include <stdio.h>

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseGoal.h>

#include <move_to/move_to.h>

#include "geometry_msgs/PoseStamped.h"
#include "ros/publisher.h"
#include "ros/time.h"

struct Node
{
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer serv;

    Node()
    {
        pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 16);
        serv = nh.advertiseService("move_to", &Node::handle_service, this);
    }

    bool handle_service(move_to::move_toRequest &req, move_to::move_toResponse &res)
    {
        ros::Time timestamp = ros::Time::now();
        geometry_msgs::PoseStamped excav_pose;
        excav_pose.header.frame_id = req.tf_name;
        excav_pose.header.seq = 1;
        excav_pose.header.stamp = timestamp;
        excav_pose.pose.orientation.w = 1;
        excav_pose.pose.orientation.x = 0;
        excav_pose.pose.orientation.y = 0;
        excav_pose.pose.orientation.z = 0;
        excav_pose.pose.position.x = 0;
        excav_pose.pose.position.y = 0;
        excav_pose.pose.position.z = 0;

        pub.publish(excav_pose);

        return true;
    }
};

int main(int argc, char **argv)
{
    Node n;
    ros::spin();
}
