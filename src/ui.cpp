#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "RT1_Assignment2/Speed_service.h"
#include "RT1_Assignment2/Speed_val.h"

ros::Publisher pub;
ros::ServiceClient service_client;

void getCommand(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    RT1_Assignment2::Speed_service service;
    char inputChar;

    std::cout << "PRESS:\n- A to accelerate\n- D to decelerate\n- R to reset position\n\n";
    std::cin >> inputChar;

    if (inputChar == 'a' || inputChar == 'd' || inputChar == 'r' || inputChar == 'A' || inputChar == 'D' || inputChar == 'R')

    {
        service.request.input_char=inputChar;
        

        service_client.waitForExistence();
        service_client.call(service);

        RT1_Assignment2::Speed_val speed;
        speed.speed = service.response.value;
        pub.publish(speed);
        system("clear");
        std::cout << "New speed: "<< service.response.value<<"\n\n";
    }
    else
    {
        system("clear");
        std::cout <<"Wrong input. Please try again.\n\n";
    }
}

int main(int argc, char **argv)
{
    system("clear");
    ros::init(argc, argv, "ui");
    ros::NodeHandle node_handle;

    service_client = node_handle.serviceClient<RT1_Assignment2::Speed_service>("/accelerator");
    ros::Subscriber subscriber = node_handle.subscribe("/base_scan", 500, getCommand);
    pub = node_handle.advertise<RT1_Assignment2::Speed_val>("/Speed_val", 500);   

    ros::spin();
    return 0;
}