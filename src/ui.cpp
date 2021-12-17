#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "RT1_Assignment2/Speed_service.h"

// ros::Publisher pub;
ros::ServiceClient service_client;

void getCommand(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    RT1_Assignment2::Speed_service service;
    char inputChar;

    //display instructions
    std::cout << "PRESS:\n- A to accelerate\n- D to decelerate\n- R to reset position\n\n";
    //wait for user input
    std::cin >> inputChar;

    if (inputChar == 'a' || inputChar == 'd' || inputChar == 'r' || inputChar == 'A' || inputChar == 'D' || inputChar == 'R')

    {
        service.request.input_char=inputChar;        
        //calling accelerator service
        service_client.waitForExistence();
        service_client.call(service);

        //update UI
        system("clear");
        std::cout << "New speed: "<< service.response.value<<"\n\n";
    }
    else
    {
        //Informs the user that the input wasn't correct
        system("clear");
        std::cout <<"Wrong input. Please try again.\n\n";
    }
}

int main(int argc, char **argv)
{
    system("clear");
    ros::init(argc, argv, "ui");
    ros::NodeHandle node_handle;

    //"linking" with accelerator node
    service_client = node_handle.serviceClient<RT1_Assignment2::Speed_service>("/accelerator");
    //subribes to /base_scan topic
    ros::Subscriber subscriber = node_handle.subscribe("/base_scan", 500, getCommand);  

    ros::spin();
    return 0;
}