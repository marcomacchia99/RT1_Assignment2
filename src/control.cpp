#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;

void callbackFnc(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    geometry_msgs::Twist vel;

    // keep array
    double min_val(double a[])
{
	double dist = 30;
	for(int i=0; i < a.size(); i++)
	{
		if(a[i] < dist)
		{
			dist = a[i];
		}
	}
	return dist;
}

    float r[msg->ranges.size()];
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        r[i] = msg->ranges[i];
    }

    double left[120];
    double front[120];
    double right[120];

    for (int i = 599; i <= 719; i++)
    {
        left[i - 599] = r[i];
    }

    for (int i = 300; i < 420; i++)
    {
        front[i - 300] = r[i];
    }

    for (int i = 0; i < 120; i++)
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
        vel.linear.x = 1.0;
    }

    pub.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Autodromo Nazionale di Monza - controller");
    ros::NodeHandle node_handle;

    ros::Subscriber sub = node_handle.subscribe("/base_scan", 500, callbackFnc);

    pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();
    return 0;
}