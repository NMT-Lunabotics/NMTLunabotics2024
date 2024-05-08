#include <can_raw/CanFrame.h>
#include <dumping_autonomy/execute_dumping_autonomy.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>

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

        service = nh.advertiseService("excavation", &State::handle_service, this);
        can_subscriber = nh.subscribe("/canbus_input", 16, &State::handle_can_msg, this);
        can_publisher = nh.advertise<can_raw::CanFrame>("/canbus", 16);

        // Spawn the thread.
        main_thread = std::thread(&State::thread_main, this);
    }

    void send_actuator_commands(int arm_vel, int bucket_vel)
    {
        can_raw::CanFrame frame;
        frame.id = 3;               // ActuatorVelCommands
        frame.data[0] = arm_vel;    // arm_vel
        frame.data[1] = bucket_vel; // bucket_vel
        can_publisher.publish(frame);
    }

    void send_drive_commands(int left_vel, int right_vel)
    {
        can_raw::CanFrame frame;
        frame.id = 1; // MotorCommands
        frame.data[0] = left_vel >> 8;
        frame.data[1] = left_vel & 0xff;
        frame.data[2] = right_vel >> 8;
        frame.data[3] = right_vel & 0xff;
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
            while (arm_pos < 250 && bucket_pos < 240)
            {
                std::cout << "Waiting for arm_pos & bucket_pos\n";
                send_actuator_commands(0, 255);
                usleep(0.1e6);
            }
            send_actuator_commands(128, 128);
            std::cout << "arm_pos & bucket_pos done, time to drive\n";

            // Now drive backwards one meter.
            send_drive_commands(0, 0);
            usleep(0.7e6);
            send_drive_commands(32768, 32768);
            std::cout << "Driving done\n";

            // And return to 100 on the arms, 60 on the bucket.
            while (arm_pos > 100 && bucket_pos > 60)
            {
                std::cout << "Getting back to retracted state\n";
                send_actuator_commands(255, 0);
                usleep(0.1e6);
            }
            std::cout << "We're done dumping!\n";
            send_actuator_commands(128, 128);

            // We're done dumping.
            dumping = false;
        }
    }

    bool handle_service(dumping_autonomy::execute_dumping_autonomyRequest &req,
                        dumping_autonomy::execute_dumping_autonomyResponse &res)
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
