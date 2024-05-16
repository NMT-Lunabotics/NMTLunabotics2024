#include <actions/execute_digging_autonomy.h>
#include <can_convert/ArmStatus.h>
#include <can_raw/CanFrame.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <thread>
#include <unistd.h>

#include "main_bus.hpp"

// State of the overall program.
struct State
{
    std::atomic<bool> digging;
    std::atomic<double> arm_pos;
    std::atomic<double> bucket_pos;

    ros::NodeHandle nh;
    ros::ServiceServer service;
    ros::Subscriber arm_status_subscriber;
    ros::Publisher can_publisher;
    ros::Publisher done_publisher;

    std::thread main_thread;

    State()
    {
        std::cout << "Digging autonomy initialized\n";

        // we get to manually initialize things here??
        digging = false;
        arm_pos = 0;
        bucket_pos = 0;

        service = nh.advertiseService("dig", &State::handle_service, this);
        arm_status_subscriber = nh.subscribe("arm_status", 16, &State::handle_arm_msg, this);
        can_publisher = nh.advertise<can_raw::CanFrame>("canbus", 16);
        done_publisher = nh.advertise<std_msgs::Empty>("digging_done", 16);

        // Spawn the thread.
        main_thread = std::thread(&State::thread_main, this);
    }

    void send_actuator_commands(double arm_vel, double bucket_vel)
    {
        can::ActuatorVelCommands cmds = {
            .arm_vel = arm_vel,
            .bucket_vel = bucket_vel,
        };

        can_raw::CanFrame frame;
        frame.id = (int)can::FrameID::ActuatorVelCommands;
        can::to_buffer(frame.data.data(), can::serialize(cmds));
        can_publisher.publish(frame);
    }

    void send_drive_commands(double left_vel, double right_vel)
    {
        can::MotorCommands cmds = {
            .left = {.speed = left_vel},
            .right = {.speed = right_vel},
        };

        can_raw::CanFrame frame;
        frame.id = (int)can::FrameID::MotorCommands;
        can::to_buffer(frame.data.data(), can::serialize(cmds));
        can_publisher.publish(frame);
    }

    double sign(double x)
    {
        if (x == 0)
            return 0;
        else if (x > 0)
            return 1;
        else
            return -1;
    }

    void set_angles(double bucket_target, double arm_target)
    {
        double bucket_delta = bucket_target - bucket_pos;
        double arm_delta = arm_target - arm_pos;

        while (bucket_delta * (bucket_target - bucket_pos) > 0 ||
               arm_delta * (arm_target - arm_pos) > 0)
        {
            double bucket_cmd = 0;
            if (bucket_delta * (bucket_target - bucket_pos) > 0)
                bucket_cmd = sign(bucket_delta) * 5;

            double arm_cmd = 0;
            if (arm_delta * (arm_target - arm_pos) > 0)
                arm_cmd = -sign(arm_delta) * 5;

            send_actuator_commands(arm_cmd, bucket_cmd);
            usleep(0.1e6);
        }
        send_actuator_commands(0, 0);
    }

    void drive(double direction, double time)
    {
        send_drive_commands(direction * 1024, direction * 1024);
        usleep(time * 1e6);
        send_drive_commands(0, 0);
    }

    void thread_main()
    {
        while (true)
        {
            // Wait until we're digging.
            usleep(0.5e6);
            if (!digging)
            {
                std::cout << "Not digging yet\n";
                continue;
            }

            // Extend the arms until they're at 250mm and the bucket is at
            // 240.
            std::cout << "Time to dig!\n";
            // set_angles(42.7, 17.1);
            set_angles(10.0, 0.0);
            drive(1, 2.0);
            // set_angles(19.0, 6.3);
            // drive(1, 1.5);
            // set_angles(-5.5, 6.3);
            set_angles(-5.5, 12.2);
            std::cout << "Done!\n";

            // We're done digging.
            std_msgs::Empty done_msg;
            done_publisher.publish(done_msg);
            digging = false;
        }
    }

    bool handle_service(actions::execute_digging_autonomyRequest &req,
                        actions::execute_digging_autonomyResponse &res)
    {
        // Hand things off to the thread.
        digging = true;
        return true;
    }

    void handle_arm_msg(const can_convert::ArmStatus &frame)
    {
        arm_pos = frame.arm_angle;
        bucket_pos = frame.bucket_angle;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "digging_autonomy");
    State state;
    ros::spin();
}
