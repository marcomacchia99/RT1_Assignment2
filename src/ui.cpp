#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "RT1_Assignment2/Speed_service.h"
#include "RT1_Assignment2/Speed_val.h"

ros::Publisher pub;

void getCommand()
{
    RT1_Assignment2::Speed_service service;
    ros::ServiceClient client;
    char inputChar;

    do
    {
        std::cout << "PRESS:\n- A to accelerate\n- D to decelerate\n -R to reset position\n\n";
        std::cin >> inputChar;
    } while (inputChar != 'a' || inputChar != 'd' || inputChar != 'r' || inputChar != 'A' || inputChar != 'D' || inputChar != 'R');
    
    service.request.input_char = inputChar;


    client.waitForExistence();
    client.call(service);

    RT1_Assignment2::Speed_val speed;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui");
    ros::NodeHandle node_handle;

    ros::ServiceClient service_client = node_handle.serviceClient<RT1_Assignment2::Speed_service>("/service");
    ros::Subscriber subscriber = node_handle.subscribe("/base_scan", 1, getCommand);

    ros::spin();
    return 0;
}