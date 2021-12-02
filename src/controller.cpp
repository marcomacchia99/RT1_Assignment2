#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define LENGTH 80

ros::Publisher pub;

double f_th = 2;

double min_val(double a[])
{
	double dist = 30;
	for(int i=0; i < LENGTH; i++)
	{
		if(a[i] < dist)
		{
			dist = a[i];
		}
	}
	return dist;
}

void callbackFnc(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    geometry_msgs::Twist vel;

    // keep arra

    float r[msg->ranges.size()];
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        r[i] = msg->ranges[i];
    }

    double left[LENGTH];
    double front[LENGTH];
    double right[LENGTH];

    for (int i = 659-((LENGTH/2)+LENGTH%2); i <= 659+((LENGTH/2)+LENGTH%2); i++)
    {
        left[i - 659-((LENGTH/2)+LENGTH%2)] = r[i];
    }

    for (int i = 360-((LENGTH/2)+LENGTH%2); i < 360+((LENGTH/2)+LENGTH%2); i++)
    {
        front[i - 360-((LENGTH/2)+LENGTH%2)] = r[i];
    }

    for (int i = 0; i < LENGTH; i++)
    {
        right[i] = r[i];
    }

    // check the distance
    if (min_val(front) < f_th)
    {
        if (min_val(right) < min_val(left))
        {
            vel.angular.z = 1;
            vel.linear.x = 0.5;
        }
        else if (min_val(right) > min_val(left))
        {
            vel.angular.z = -1;
            vel.linear.x = 0.5;
        }
    }
    else if (min_val(front) > f_th)
    {
        vel.linear.x = 5.0;
    }

    pub.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle node_handle;

    ros::Subscriber sub = node_handle.subscribe("/base_scan", 500, callbackFnc);

    pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();
    return 0;
}