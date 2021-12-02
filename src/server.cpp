#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "RT1_Assignment2/Speed_service.h"
#include "std_srvs/Empty.h"

std_srvs::Empty reset;

float variation = 0;

bool setSpeed(RT1_Assignment2::Speed_service::Request &req, RT1_Assignment2::Speed_service::Response &res)
{
    switch (req.input_char)
    {
    case 'a':
    case 'A':
        variation+=0.5;
        break;
    case 'd':
    case 'D':
        variation-=0.5;
        break;
    case 'r':
    case 'R':
        ros::service::call("/reset_position", reset);
        break;
    default:
        break;
    }
    res.value = variation;
    return true;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("/accelerator", setSpeed);

    ros::spin();
    return 0;
}