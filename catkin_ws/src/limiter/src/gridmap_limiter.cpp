#include <iostream>
#include <stdio.h>

#include "limiter.hpp"
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage: limiter <name> <topic>\n");
        return 1;
    }

    std::string name = "pointcloud_limiter";
    ros::init(argc, argv, name + argv[1]);

    Limiter<grid_map_msgs::GridMap> l(argv[2]);
    ros::spin();
}
