// Fuck it. Full autonomy.

#include "ros/service.h"
#include "std_msgs/Empty.h"
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>

#include <actions/execute_digging_autonomy.h>
#include <actions/execute_dumping_autonomy.h>
#include <actions/fuckit.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_to/move_to.h>

struct State
{
    ros::NodeHandle nh;
    ros::ServiceServer service;
    ros::Subscriber dig_done_subscribero;
    ros::Subscriber move_base_subscriber;

    std::thread main_thread;
    std::atomic<bool> going;
    std::atomic<bool> done_digging;
    std::atomic<bool> done_navigating;

    State()
    {
        going = false;
        done_digging = false;
        done_navigating = false;

        service = nh.advertiseService("fuckit", &State::handle_service, this);
        dig_done_subscribero = nh.subscribe("digging_done", 16, &State::handle_dig_done, this);
        move_base_subscriber =
            nh.subscribe("/move_base/result", 10, &State::handle_move_base_done, this);

        main_thread = std::thread(&State::thread_main, this);
    }

    void thread_main()
    {
        std::cout << "thread_main()\n";
        while (true)
        {
            usleep(0.5e6);
            if (!going)
            {
                std::cout << "Not full autonomy yet\n";
                continue;
            }

            std::cout << "Fuck it, full autonomy baybee\n";

            nav_to_dig();
            std::cout << "At dig\n";
            dig();
            std::cout << "Digging done, scheduling dump\n";
            schedule_dump();
            nav_to_dump();
            std::cout << "Done dumping, let's exit\n";
            return;
        }
    }

    void nav_to_dig()
    {
        done_navigating = false;
        move_to::move_to svc;
        svc.request.tf_name = "excavation_center";
        while (!done_navigating)
            usleep(0.1e6);
    }

    void nav_to_dump()
    {
        done_navigating = false;
        move_to::move_to svc;
        svc.request.tf_name = "dump_center";
        while (!done_navigating)
            usleep(0.1e6);
    }

    void dig()
    {
        done_digging = false;
        actions::execute_digging_autonomy action;
        ros::service::call("dig", action);
        while (!done_digging)
            usleep(0.1e6);
    }

    void schedule_dump()
    {
        actions::execute_dumping_autonomy action;
        ros::service::call("dump", action);
    }

    // ---- handlers ----

    void handle_dig_done(const std_msgs::Empty &emp)
    {
        done_digging = true;
    }

    void handle_move_base_done(const move_base_msgs::MoveBaseActionResult::ConstPtr &result)
    {
        done_navigating = true;
    }

    bool handle_service(actions::fuckitRequest &req, actions::fuckitResponse &resp)
    {
        std::cout << "Service received\n";
        going = true;
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fuckit");
    State state;
    ros::spin();
}
