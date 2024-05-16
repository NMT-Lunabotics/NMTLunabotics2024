#include <iostream>
#include <stdio.h>

#include "limiter.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_limiter");

    if (argc != 2)
    {
        fprintf(stderr, "usage: limiter <topic>\n");
        return 1;
    }

    Limiter<sensor_msgs::PointCloud2> l(argv[1]);
    ros::spin();
}
