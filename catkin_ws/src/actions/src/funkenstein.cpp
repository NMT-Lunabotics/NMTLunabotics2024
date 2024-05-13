// Funkenstein language

#include <actionlib/client/simple_action_client.h>
#include <actions/execute_script.h>
#include <can_raw/CanFrame.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>

#include "main_bus.hpp"

struct State
{
    ros::NodeHandle nh;
    ros::ServiceServer service;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    ros::Subscriber can_subscriber;
    ros::Publisher can_publisher;

    double arm_pos = 0;
    double bucket_pos = 0;

    State()
    {
        nh.advertiseService("funk", &State::handle_service, this);
        can_subscriber = nh.subscribe("/canbus_input", 16, &State::handle_can_msg, this);
        can_publisher = nh.advertise<can_raw::CanFrame>("/canbus", 16);
    }

    bool handle_service(actions::execute_scriptRequest &req, actions::execute_scriptResponse &res)
    {
        const std::string &script_name = req.script_name;
        std::string script = read_to_file(script_name);
        std::vector<std::string> script_lines = lines(script, '\n');

        return true;
    }

    static std::string read_to_file(const std::string &filename)
    {
        FILE *script_file = fopen(filename.c_str(), "r");
        assert(script_file);
        fseek(script_file, 0, SEEK_END);
        size_t script_len = ftell(script_file);

        std::string script;
        script.resize(script_len);
        fread((void *)script.data(), 1, script_len, script_file);
        fclose(script_file);

        return script;
    }

    static std::vector<std::string> lines(const std::string &str, char by)
    {
        int line_start = 0;
        int line_end = 0;

        std::vector<std::string> result;
        while (line_end < str.size())
        {
            if (str[line_end] == by)
            {
                result.push_back(str.substr(line_start, line_end));
                line_start = line_end + 1;
            }

            line_end++;
        }

        if (line_start != line_end)
            result.push_back(str.substr(line_start, line_end));

        return result;
    }

    void do_lines(std::vector<std::string> coke)
    {
        for (std::string line : coke)
        {
            // Comments & blanks.
            if (line[0] == '#' || line.size() == 0)
                continue;

            std::istringstream iss(line);
            std::string command;
            int number;
            std::vector<int> numbers;

            // Read the first word from the string
            iss >> command;

            // Read the remaining numbers from the string
            while (iss >> number)
            {
                numbers.push_back(number);
            }

            // Process based on the keyword
            if (command == "actuator")
            {
                if (numbers.size() != 2)
                {
                    // TODO throw an error or somethign
                }
                can::ActuatorVelCommands cmds = {
                    .arm_vel = (double)numbers[0],
                    .bucket_vel = (double)numbers[1],
                };

                can_raw::CanFrame frame;
                frame.id = (int)can::FrameID::ActuatorVelCommands;
                can::to_buffer(frame.data.data(), can::serialize(cmds));
                can_publisher.publish(frame);
            }
            else if (command == "motor")
            {
                if (numbers.size() != 2)
                {
                    // TODO throw an error or somethign
                }
                can::MotorCommands cmds = {
                    .left = {.speed = (double)numbers[0]},
                    .right = {.speed = (double)numbers[1]},
                };

                can_raw::CanFrame frame;
                frame.id = (int)can::FrameID::MotorCommands;
                can::to_buffer(frame.data.data(), can::serialize(cmds));
                can_publisher.publish(frame);
            }
            else if (command == "move_base")
            {
                if (numbers.size() != 3)
                {
                    // TODO throw an error or somethign
                }
                // Create the action client
                MoveBaseClient ac("move_base", true);

                // Wait for the action server to start
                ROS_INFO("Waiting for the move_base action server");
                ac.waitForServer();

                // Define a goal to send to move_base
                move_base_msgs::MoveBaseGoal goal;

                // Use the map frame to define goal coordinates
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();

                // Convert yaw to quaternion
                double yaw = numbers[2] * M_PI / 180;
                geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(yaw);

                // Define position and orientation for the goal (x, y, and orientation
                // in quaternion)
                goal.target_pose.pose.position.x = numbers[0];
                goal.target_pose.pose.position.y = numbers[1];
                goal.target_pose.pose.orientation = yaw_quat;

                // Send the goal
                ROS_INFO("Sending goal");
                ac.sendGoal(goal);

                // Wait for the action to return
                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("The base moved to goal");
                else
                    ROS_INFO("The base failed to move to goal for some reason");
            }
        }
    }

    static void goalReachedCallback(const actionlib::SimpleClientGoalState &state,
                                    const move_base_msgs::MoveBaseResult::ConstPtr &result)
    {
        ROS_INFO("Goal reached!");
        return;
    }

    void handle_can_msg(const can_raw::CanFrame &frame)
    {
        if (frame.id == 4)
        {
            // ActuatorArmPos
            int left_pos = frame.data[0];
            int right_pos = frame.data[1];
            arm_pos = (left_pos + right_pos) / 2;
        }
        else if (frame.id == 5)
        {
            // ActuatorBucketPos
            bucket_pos = frame.data[0];
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "funkenstein");
    State state;
    ros::spin();
}
