#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_markers");
    ros::NodeHandle nh;
    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(10);

    // Define the 2D array of line coordinates
    float berm_x = 5.38;
    float berm_y = 0.6;
    float berm_width = 2;
    float berm_height = 0.7;

    float column_size = 0.35;
    float column_x = 3.88 - column_size / 2;
    float column_y = 2.5;

    float lines[][4] = {
        {3.88, 0, 3.88, 5},
        {3.88, 2, 6.88, 2},
        {0, 0, 6.88, 0},
        {6.88, 0, 6.88, 5},
        {6.88, 5, 0, 5},
        {0, 5, 0, 0},
        {berm_x - berm_width / 2, berm_y - berm_height / 2,
            berm_x + berm_width / 2, berm_y - berm_height / 2},
        {berm_x + berm_width / 2, berm_y - berm_height / 2,
            berm_x + berm_width / 2, berm_y + berm_height / 2},
        {berm_x + berm_width / 2, berm_y + berm_height / 2,
            berm_x - berm_width / 2, berm_y + berm_height / 2},
        {berm_x - berm_width / 2, berm_y + berm_height / 2,
            berm_x - berm_width / 2, berm_y - berm_height / 2},
        {column_x - column_size / 2, column_y - column_size / 2,
            column_x + column_size / 2, column_y - column_size / 2},
        {column_x + column_size / 2, column_y - column_size / 2,
            column_x + column_size / 2, column_y + column_size / 2},
        {column_x + column_size / 2, column_y + column_size / 2,
            column_x - column_size / 2, column_y + column_size / 2},
        {column_x - column_size / 2, column_y + column_size / 2,
            column_x - column_size / 2, column_y - column_size / 2},
    };

    // Number of lines
    const int num_lines = sizeof(lines) / sizeof(lines[0]);

    while (ros::ok())
    {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // Line width
        line_list.scale.x = 0.02;

        // Line color (red)
        line_list.color.r = 1.0;
        line_list.color.a = 0.5;

        // Loop over the lines array and add points to the marker
        for (int i = 0; i < num_lines; ++i)
        {
            geometry_msgs::Point p1, p2;
            p1.x = lines[i][0];
            p1.y = lines[i][1];
            p1.z = 0;
            p2.x = lines[i][2];
            p2.y = lines[i][3];
            p2.z = 0;
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
        }

        marker_pub.publish(line_list);

        r.sleep();
    }

    return 0;
}
