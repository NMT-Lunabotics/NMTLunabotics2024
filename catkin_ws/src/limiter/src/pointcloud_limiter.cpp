#include <iostream>
#include <stdio.h>

#include "limiter.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage: limiter <name> <topic>\n");
        return 1;
    }

    std::string name = "pointcloud_limiter";
    ros::init(argc, argv, name + argv[1]);

    Limiter<sensor_msgs::PointCloud2> l(argv[2]);
    ros::spin();
}
