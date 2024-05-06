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
        // Initialize parameters
        nh_.param("goal_x", goal_x_, 10.0);
        nh_.param("goal_y", goal_y_, 10.0);
        nh_.param("goal_yaw", goal_yaw_, 180.0);
        nh_.param("dump_button", dump_button_, 0);

        // Wait for the action server to come up
        while(!ac_.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        joy_sub_ = nh_.subscribe("joy_teleop/joy", 10, &DumpNode::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        // Check if the configured button is pressed
        if (joy->buttons[dump_button_] == 1)
        {
            sendGoal(goal_x_, goal_y_, goal_yaw_);
        }
    }

    void sendGoal(double x, double y, double yaw_degrees)
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
        std_srvs::Empty srv;
        if (ros::service::call("Dump", srv))
            ROS_INFO("Successfully called Dump service.");
        else
            ROS_ERROR("Failed to call Dump service.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    double goal_x_, goal_y_, goal_yaw_;
    int dump_button_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dump_node");
    DumpNode dump_node;
    ros::spin();
    return 0;
}
