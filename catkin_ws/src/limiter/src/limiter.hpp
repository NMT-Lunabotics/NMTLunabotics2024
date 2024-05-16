#ifndef LIMITER_H
#define LIMITER_H

#include <ros/ros.h>
#include <stdio.h>

template <typename T> struct Limiter
{
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer;

    bool have_data = false;
    T last;

    Limiter(const std::string &topic_name)
    {
        sub = nh.subscribe(topic_name, 512, &Limiter::handle_msg, this);
        pub = nh.advertise<T>(topic_name + "_limited", 512);
        timer = nh.createTimer(ros::Duration(0.25), &Limiter::handle_timer, this);
    }

    void handle_msg(const T &msg)
    {
        last = msg;
        have_data = true;
    }

    void handle_timer(const ros::TimerEvent &ev)
    {
        if (have_data)
        {
            printf("publishing real data\n");
            pub.publish(last);
        }
        else
        {
            printf("no data yet\n");
        }
    }
};

#endif /* LIMITER_H */
