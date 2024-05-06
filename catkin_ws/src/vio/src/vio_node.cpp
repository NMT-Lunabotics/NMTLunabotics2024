#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image from %s", msg->header.frame_id.c_str());
    // Process the image here...
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vio_node");
    ros::NodeHandle nh("~");

    std::string topic_name;
    nh.param<std::string>("topic_name", topic_name, "/image");

    ros::Subscriber image_subscriber = nh.subscribe(topic_name, 1, imageCallback);

    ros::spin();

    return 0;
}
