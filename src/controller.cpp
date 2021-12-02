#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "RT1_Assignment2/Speed_service.h"

#define LENGTH 80

ros::Publisher publisher;

double f_th = 2;
float lin_vel = 5;

double min(double a[])
{
    double min = 30;
    for (int i = 0; i < LENGTH; i++)
    {
        if (a[i] < min)
            min = a[i];
    }
    return min;
}

void checkTrackLimits(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    geometry_msgs::Twist vel;

    float r[msg->ranges.size()];
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        r[i] = msg->ranges[i];
    }

    double left[LENGTH];
    double front[LENGTH];
    double right[LENGTH];

    for (int i = 719 - LENGTH; i <= 719; i++)
    {
        left[i - (719 - LENGTH)] = r[i];
    }

    for (int i = 360 - ((LENGTH / 2) + LENGTH % 2); i < 360 + ((LENGTH / 2) + LENGTH % 2); i++)
    {
        front[i - (360 - ((LENGTH / 2) + LENGTH % 2))] = r[i];
    }

    for (int i = 0; i < LENGTH; i++)
    {
        right[i] = r[i];
    }

    if (min(front) < f_th)
    {
        if (min(right) < min(left))
        {
            vel.angular.z = 2;
            vel.linear.x = 1;
        }
        else if (min(right) > min(left))
        {
            vel.angular.z = -2;
            vel.linear.x = 1;
        }
    }
    else if (min(front) > f_th)
    {
        vel.linear.x = lin_vel;
    }

    publisher.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle node_handle;

    ros::Subscriber subscriber = node_handle.subscribe("/base_scan", 500, checkTrackLimits);

    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();
    return 0;
}