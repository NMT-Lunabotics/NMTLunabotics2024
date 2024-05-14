#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "create_occupancy");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    ros::Rate loop_rate(10); // Publishing at 10 Hz

    // Parameters for grid
    double width, height, resolution;
    nh.param("grid_width", width, 10.0);          // Default width 10 meters
    nh.param("grid_height", height, 10.0);        // Default height 10 meters
    nh.param("grid_resolution", resolution, 0.1); // Default resolution 0.1 meters

    // Prepare the occupancy grid
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = resolution;
    grid.info.width = static_cast<unsigned int>(width / resolution);
    grid.info.height = static_cast<unsigned int>(height / resolution);
    grid.info.origin.position.x = width / 2;
    grid.info.origin.position.y = height / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(grid.info.width * grid.info.height, 0); // 0 indicates fully traversable

    while (ros::ok())
    {
        grid.header.stamp = ros::Time::now();
        map_pub.publish(grid);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
