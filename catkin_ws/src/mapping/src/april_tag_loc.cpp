#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

static bool have_transform = false;
static tf::Transform map_to_t265odom;

// Get the transforms that will be added together (i.e. d435 and t265)
bool getTransform(const std::string &from_frame, const std::string &to_frame,
                  tf::StampedTransform &transform, const tf::TransformListener &listener)
{
    // Wait until the transform is available
    try
    {
        listener.waitForTransform(from_frame, to_frame, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(from_frame, to_frame, ros::Time(0), transform);
        return true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

// Broadcast the new apriltag to t265 odom frame
void calculate_tf(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg,
                  const tf::TransformListener &listener, tf::TransformBroadcaster &broadcaster)
{
    // If the detection is found
    if (!msg->detections.empty())
    {
        tf::StampedTransform tag_to_map;
        getTransform("map", "tag", tag_to_map, listener);
        tf::StampedTransform odom_to_tag;
        if (getTransform("t265_odom_frame", "tag_righted", odom_to_tag, listener))
        {
            tf::StampedTransform t265odom_to_d435;
            getTransform("t265_odom_frame", "d435_color_optical_frame", t265odom_to_d435, listener);

            auto odom_to_map = odom_to_tag * tag_to_map;
            map_to_t265odom = odom_to_map.inverse();

            have_transform = true;
        }
    }
}

void broadcast_tf(tf::TransformBroadcaster &broadcaster)
{
    if (have_transform)
    {
        std::cout << "Broadcasting transform\n";

        broadcaster.sendTransform(
            tf::StampedTransform(map_to_t265odom, ros::Time::now(), "map", "t265_odom_frame"));
    }
}

struct callback_data
{
    tf::TransformListener &listener;
    tf::TransformBroadcaster &broadcaster;

    void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &a)
    {
        calculate_tf(a, listener, broadcaster);
    }

    void timerCallback(const ros::TimerEvent &ev)
    {
        broadcast_tf(broadcaster);
    }
};

// Initialize the node "tag_transform_handler"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_transform_handler");
    ros::NodeHandle node;

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    callback_data data = {.listener = listener, .broadcaster = broadcaster};

    // Subscribe to AprilTag detections
    ros::Subscriber sub =
        node.subscribe("tag_detections", 1, &callback_data::aprilTagCallback, &data);

    // Set a timer to republish transform.
    ros::Timer timer = node.createTimer(10, &callback_data::timerCallback, &data);

    ros::spin();
    return 0;
}
