#include <digging_autonomy/execute_digging_autonomy.h>
#include <ros/ros.h>

bool handle_service(digging_autonomy::execute_digging_autonomyRequest &req,
                    digging_autonomy::execute_digging_autonomyResponse &res)
{
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "digging_autonomy");

    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("excavation", handle_service);
}
