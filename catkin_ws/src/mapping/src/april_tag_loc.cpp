#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

static bool have_transform = false;
static tf::Transform transform;

// Broadcast the new apriltag to t265 odom frame
void calculate_tf(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg,
                  const tf::TransformListener &listener, tf::TransformBroadcaster &broadcaster)
{
    // If the detection is found
    if (!msg->detections.empty())
    {
        // Set first april tag detected pose
        const auto &detection = msg->detections[0];

        // Transform the detected apriltag pose into a tf and add it to the cameras-tf
        tf::Pose tag_transform;
        tf::poseMsgToTF(detection.pose.pose.pose, tag_transform);
        have_transform = true;
        transform = tag_transform;
    }
}

void broadcast_tf(tf::TransformBroadcaster &broadcaster)
{
    if (have_transform)
    {
        std::cout << "Broadcasting transform\n";

        broadcaster.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "d435_link", "map"));
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
