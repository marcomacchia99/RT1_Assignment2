#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "RT1_Assignment2/Speed_service.h"
#include "std_srvs/Empty.h"

std_srvs::Empty reset;

float speed = 1;

bool setSpeed(RT1_Assignment2::Speed_service::Request &req, RT1_Assignment2::Speed_service::Response &res)
{


    // char buffer[80];
    // strcpy(buffer, req.input_char.c_str());
    // // int input = atoi(req.input_char.c_str());
    // // int input = (int)req.input_char;

    // printf("string: >%s<\n",req.input_char.c_str());
    // printf("input: %d\n", (int)(buffer[0]));
    // char input = buffer[0];
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
    speed=0;
    ros::init(argc, argv, "server");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("/accelerator", setSpeed);

    ros::spin();
    return 0;
}