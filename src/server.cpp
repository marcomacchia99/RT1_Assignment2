#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "RT1_Assignment2/Speed_service.h"
#include "std_srvs/Empty.h"

std_srvs::Empty reset;

float speed = 1;

bool setSpeed(RT1_Assignment2::Speed_service::Request &req, RT1_Assignment2::Speed_service::Response &res)
{
    switch (req.input_char)
    {
    case 'a':
    case 'A':
        speed += 0.5;
        break;
    case 'd':
    case 'D':
        if(speed>=0.5)
        speed += -0.5;
        break;
    case 'r':
    case 'R':
        speed = 1;
        ros::service::call("/reset_positions", reset);
        break;
    default:
        break;
    }
    res.value = speed;

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