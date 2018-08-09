#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/LightSensorValues.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerResponse.h"
#include <fstream>
#include "signal.h"
using namespace ros;

/* wall_around.cpp
 * Copyright (c) 2018 Ryuichi Ueda <ryuichiueda@gmail.com>
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php */

int sensor_left_side = 0;
int sensor_right_side = 0;
int sensor_left_forward = 0;
int sensor_right_forward = 0;

void callback(const raspimouse_ros_2::LightSensorValues::ConstPtr& msg)
{
	sensor_left_side = msg->left_side;
	sensor_right_side = msg->right_side;
	sensor_left_forward = msg->left_forward;
	sensor_right_forward = msg->right_forward;
}

void onSigint(int sig)
{
	std_srvs::Trigger trigger;
	service::call("/motor_off", trigger);
	shutdown();
}

bool wall_front(void)
{
	return sensor_left_forward > 50 or sensor_right_forward > 50;
}

bool too_right(void)
{
	return sensor_right_side > 50;
}

bool too_left(void)
{
	return sensor_left_side > 50;
}

void run(Publisher *pub)
{
	geometry_msgs::Twist tw;
	tw.linear.x = 0.3;

	if(wall_front())
		tw.angular.z = -3.141592;
	else if(too_right())
		tw.angular.z = 3.141592;
	else if(too_left())
		tw.angular.z = -3.141592;
	else{
		double e = 50.0 - sensor_left_side;
		tw.angular.z = e*3.141592/180;
	}

	pub->publish(tw);
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

	while(ok()){
		run(&pub);
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}
