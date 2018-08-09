#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/LightSensorValues.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerResponse.h"
#include <fstream>
#include "signal.h"
using namespace ros;

/* wall_stop.cpp
 * Copyright (c) 2018 Ryuichi Ueda <ryuichiueda@gmail.com>
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php */

int sensor_sum_all = 0;

void callback(const raspimouse_ros_2::LightSensorValues::ConstPtr& msg)
{
	sensor_sum_all = msg->sum_all;
}

void onSigint(int sig)
{
	std_srvs::Trigger trigger;
	service::call("/motor_off", trigger);
	shutdown();
}

void run(Publisher *pub, geometry_msgs::Twist *tw)
{
        const double accel = 0.02;
	tw->linear.x += accel;

	if(sensor_sum_all >= 50)
		tw->linear.x = 0.0;
	else if(tw->linear.x <= 0.2)
		tw->linear.x = 0.2;
	else if(tw->linear.x >= 0.8)
		tw->linear.x = 0.8;

	pub->publish(*tw);
}

int main(int argc, char **argv)
{
	init(argc,argv,"wall_stop");
	NodeHandle n("~");
	
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	Subscriber sub = n.subscribe("/lightsensors", 1, callback);

	service::waitForService("/motor_on");
	service::waitForService("/motor_off");

	signal(SIGINT, onSigint);

	std_srvs::Trigger trigger;
	service::call("/motor_on", trigger);

	int freq = 10;

	ros::Rate loop_rate(freq);
	raspimouse_ros_2::LightSensorValues msg;

	geometry_msgs::Twist tw;
	tw.linear.x = 0.0;
	tw.angular.z = 0.0;
	while(ok()){
		run(&pub,&tw);
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}
