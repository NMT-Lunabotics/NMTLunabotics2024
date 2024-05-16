#include <iostream>
#include <stdio.h>

#include "limiter.hpp"
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gridmap_limiter");

    if (argc != 2)
    {
        fprintf(stderr, "usage: limiter <topic>\n");
        return 1;
    }

    Limiter<grid_map_msgs::GridMap> l(argv[1]);
    ros::spin();
}
