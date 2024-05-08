#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>

class DumpNode
{
public:
    DumpNode() : ac_("move_base", true)
    {
        // Wait for the action server to come up
        while(!ac_.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        joy_sub_ = nh_.subscribe("joy_teleop/joy", 10, &DumpNode::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        // Check if the button (index 0) is pressed
        if (joy->buttons[0] == 1)
        {
            sendGoal(10.0, 10.0, 180.0);
        }
    }

    void sendGoal(float x, float y, float yaw_degrees)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        // Convert yaw to quaternion
        double yaw = yaw_degrees * M_PI/180;
        geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(yaw);

        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation = yaw_quat;

        // Send the goal
        ac_.sendGoal(goal, boost::bind(&DumpNode::goalReachedCallback, this, _1, _2));
    }

    void goalReachedCallback(const actionlib::SimpleClientGoalState& state,
                             const move_base_msgs::MoveBaseResult::ConstPtr& result)
    {
        ROS_INFO("Goal reached!");
        callDumpService();
    }

    void callDumpService()
    {
        std_srvs::dump srv;
        if (ros::service::call("dump", srv))
            ROS_INFO("Successfully called Dump service.");
        else
            ROS_ERROR("Failed to call Dump service.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dump_node");
    DumpNode dump_node;
    ros::spin();
    return 0;
}
