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
		service::waitForService("/motor_on");
		service::waitForService("/motor_off");
		signal(SIGINT, onSigint);
		std_srvs::Trigger trigger;
		service::call("/motor_on", trigger);
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
	
		if(sensor_sum_all >= 500)
			tw.linear.x = 0.0;

		pub.publish(tw);
	}

private:
	int sensor_sum_all = 0;
	NodeHandle n = NodeHandle("~");
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	Subscriber sub = n.subscribe("/lightsensors", 1, &WallStop::callback, this);
};

int main(int argc, char **argv)
{
	init(argc,argv,"wall_stop");
	WallStop w;

	int freq = 10;
	ros::Rate loop_rate(freq);

	while(ok()){
		w.run();
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}

