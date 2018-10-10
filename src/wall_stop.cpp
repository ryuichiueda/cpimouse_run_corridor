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

void onSigint(int sig)
{
	std_srvs::Trigger trigger;
	service::call("/motor_off", trigger);
	shutdown();
}

class WallStop
{
public:
	WallStop(){
		n = NodeHandle("~");
	
		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		sub = n.subscribe("/lightsensors", 1, &WallStop::callback, this);

		service::waitForService("/motor_on");
		service::waitForService("/motor_off");
	}

	void callback(const raspimouse_ros_2::LightSensorValues::ConstPtr& msg)
	{
		sensor_sum_all = msg->sum_all;
	}

	void run()
	{
		geometry_msgs::Twist tw;
		tw.linear.x = 0.2;
		tw.angular.z = 0.0;
	
		if(sensor_sum_all >= 500){
			tw.linear.x = 0.0;
		}
		pub.publish(tw);
	}

private:
	int sensor_sum_all;
	NodeHandle n;
	Publisher pub;
	Subscriber sub;
};

int main(int argc, char **argv)
{
	init(argc,argv,"wall_stop");
	WallStop w = WallStop();

	signal(SIGINT, onSigint);

	std_srvs::Trigger trigger;
	service::call("/motor_on", trigger);

	int freq = 10;

	ros::Rate loop_rate(freq);
	raspimouse_ros_2::LightSensorValues msg;

	unsigned int c = 0;
	while(ok()){
		w.run();
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}

