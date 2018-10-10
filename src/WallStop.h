#ifndef __WALL_STOP_
#define __WALL_STOP_

#include "ros/ros.h"
#include <ros/package.h> 
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerResponse.h"
#include "raspimouse_ros_2/LightSensorValues.h"
using namespace ros;

class WallStop
{
public:
	WallStop();
	void callback(const raspimouse_ros_2::LightSensorValues::ConstPtr& msg);
	void run();
	static void onSigint(int sig);
	
private:
	int sensor_sum_all = 0;
	NodeHandle n = NodeHandle("~");
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	Subscriber sub = n.subscribe("/lightsensors", 1, &WallStop::callback, this);
};

#endif
