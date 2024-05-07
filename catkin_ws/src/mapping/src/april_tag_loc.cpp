#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

static bool have_transform = false;
static tf::Transform map_to_tag;

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
        tf::StampedTransform map_to_d435;
        if (getTransform("map", "d435_link", map_to_d435, listener))
        {
            // Set first april tag detected pose
            const auto &detection = msg->detections[0];

            // Transform the detected apriltag pose into a tf and add it to the cameras-tf
            tf::Pose d435_to_tag;
            tf::poseMsgToTF(detection.pose.pose.pose, d435_to_tag);
            have_transform = true;
            map_to_tag = map_to_d435 * d435_to_tag;
        }
    }
}

void broadcast_tf(tf::TransformBroadcaster &broadcaster)
{
    if (have_transform)
    {
        std::cout << "Broadcasting transform\n";

        broadcaster.sendTransform(tf::StampedTransform(map_to_tag, ros::Time::now(), "map", "tag"));
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
