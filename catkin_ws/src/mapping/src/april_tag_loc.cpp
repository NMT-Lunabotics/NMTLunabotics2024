#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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
void broadTF(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg,
             const tf::TransformListener &listener, tf::TransformBroadcaster &broadcaster)
{
    tf::StampedTransform d435_to_t265;
    // If the detection is found
    if (!msg->detections.empty())
    {
        // Get the tranform between the d435 and t265
        if (getTransform("d435_link", "t265_link", d435_to_t265, listener))
        {

            // Set first april tag detected pose
            const auto &detection = msg->detections[0];

            // Transform the detected apriltag pose into a tf and add it to the cameras-tf
            tf::Pose tag_transform;
            tf::poseMsgToTF(detection.pose.pose.pose, tag_transform);
            tf::Transform adjusted_transform = d435_to_t265 * tag_transform;

            // Broadcast new transform from "map" to "t265_odom_frame"
            broadcaster.sendTransform(tf::StampedTransform(adjusted_transform, ros::Time::now(),
                                                           "map", "t265_odom_frame"));
        }
    }
}

struct callback_data
{
    tf::TransformListener &listener;
    tf::TransformBroadcaster &broadcaster;

    void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &a)
    {
        broadTF(a, listener, broadcaster);
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

    ros::spin();
    return 0;
}
