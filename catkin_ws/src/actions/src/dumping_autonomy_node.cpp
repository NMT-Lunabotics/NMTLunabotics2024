#include <actions/execute_dumping_autonomy.h>
#include <can_raw/CanFrame.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>

#include "main_bus.hpp"

// State of the overall program.
struct State
{
    std::atomic<bool> dumping;
    std::atomic<int> arm_pos;
    std::atomic<int> bucket_pos;

    ros::NodeHandle nh;
    ros::ServiceServer service;
    ros::Subscriber can_subscriber;
    ros::Publisher can_publisher;

    std::thread main_thread;

    State()
    {
        std::cout << "Dumping autonomy initialized\n";

        // we get to manually initialize things here??
        dumping = false;
        arm_pos = 0;
        bucket_pos = 0;

        service = nh.advertiseService("dump", &State::handle_service, this);
        can_subscriber = nh.subscribe("/canbus_input", 16, &State::handle_can_msg, this);
        can_publisher = nh.advertise<can_raw::CanFrame>("/canbus", 16);

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

    void thread_main()
    {
        while (true)
        {
            // Wait until we're dumping.
            usleep(0.5e6);
            if (!dumping)
            {
                std::cout << "Not dumping yet\n";
                continue;
            }

            // Extend the arms until they're at 250mm and the bucket is at
            // 240.
            std::cout << "Time to dump!\n";
            while (arm_pos < 250 || bucket_pos < 200)
            {
                std::cout << "Waiting for arm_pos & bucket_pos\n";
                send_actuator_commands((arm_pos < 250) ? -5 : 0, (bucket_pos < 200) ? 5 : 0);
                usleep(0.1e6);
            }
            send_actuator_commands(0, 0);
            std::cout << "arm_pos & bucket_pos done, time to drive\n";

            // Now drive backwards one meter.
            send_drive_commands(-1024, -1024);
            usleep(0.7e6);
            send_drive_commands(0, 0);
            std::cout << "Driving done\n";

            // And return to 100 on the arms, 60 on the bucket.
            while (arm_pos > 100 || bucket_pos > 60)
            {
                std::cout << "Getting back to retracted state\n";
                send_actuator_commands((arm_pos > 100) ? 5 : 0, (bucket_pos > 60) ? -5 : 0);
                usleep(0.1e6);
            }
            std::cout << "We're done dumping!\n";
            send_actuator_commands(0, 0);

            // We're done dumping.
            dumping = false;
        }
    }

    bool handle_service(actions::execute_dumping_autonomyRequest &req,
                        actions::execute_dumping_autonomyResponse &res)
    {
        // Hand things off to the thread.
        dumping = true;
        return true;
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
    ros::init(argc, argv, "dumping_autonomy");
    State state;
    ros::spin();
}
