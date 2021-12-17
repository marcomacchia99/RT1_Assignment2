#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "RT1_Assignment2/Speed_service.h"
#include "std_srvs/Empty.h"

#define LENGTH 80

ros::Publisher publisher;
std_srvs::Empty reset;

double f_th = 2;
float speed = 1;

/**  Returns the minumim value of a given array  */
double min(double a[])
{
    double min = 100;
    for (int i = 0; i < LENGTH; i++)
    {
        if (a[i] < min)
            min = a[i];
    }
    return min;
}

/** @brief Check all the visible walls.
 *
 * This function checks all the walls that the robot can see. It uses the robot scanner, and it checks
 * all the blocks that are in front of the robot, at his left and at his right.
 * If the robot is close to a wall, it makes him turn, otherwise it makes him go straight.
 *
 * @param msg The message published into /base_scan topic
 *
 */
void checkTrackLimits(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    geometry_msgs::Twist vel;

    double left[LENGTH];
    double front[LENGTH];
    double right[LENGTH];

    //the last LENGTH element of the arrays refers to the walls on the left
    for (int i = 719 - LENGTH; i <= 719; i++)
    {
        left[i - (719 - LENGTH)] = msg->ranges[i];
    }

    //the mid-LENGTH element of the arrays refers to the walls in front of the robot
    for (int i = 360 - ((LENGTH / 2) + LENGTH % 2); i < 360 + ((LENGTH / 2) + LENGTH % 2); i++)
    {
        front[i - (360 - ((LENGTH / 2) + LENGTH % 2))] = msg->ranges[i];
    }

    //the first LENGTH element of the arrays refers to the walls on the right
    for (int i = 0; i < LENGTH; i++)
    {
        right[i] = msg->ranges[i];
    }

    //if the nearest wall in front of the robot is close, the robot turns...
    if (min(front) < f_th)
    {
        //...to the left or...
        if (min(right) < min(left))
        {
            vel.angular.z = 1.5;
            vel.linear.x = 1;
        }
        //...to the right
        else if (min(right) > min(left))
        {
            vel.angular.z = -1.5;
            vel.linear.x = 1;
        }
    }
    //otherwise the robot goes straight
    else if (min(front) >= f_th)
    {
        vel.linear.x = speed;
    }

    //publish the new speed to the relative topic
    publisher.publish(vel);
}

/**  Update the current robot speed based on the command generated from UI service  */
bool setSpeed(RT1_Assignment2::Speed_service::Request &req, RT1_Assignment2::Speed_service::Response &res)
{
    switch (req.input_char)
    {
    case 'a':
    case 'A':
        //increase speed
        speed += 0.5;
        break;
    case 'd':
    case 'D':
        //decrease speed
        if (speed >= 0.5)
            speed += -0.5;
        break;
    case 'r':
    case 'R':
        //reset position and speed
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
    ros::init(argc, argv, "controller");
    ros::NodeHandle node_handle;

    //receive wall scan updated by subscribing at /base_scan topic
    ros::Subscriber subScan = node_handle.subscribe("/base_scan", 500, checkTrackLimits);
    //telling ros that there's a new service available managed by controller
    ros::ServiceServer service = node_handle.advertiseService("/accelerator", setSpeed);
    //this service will publish updated into /cmd_vel topic
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();
    return 0;
}