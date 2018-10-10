#include "signal.h"
#include "WallStop.h"
using namespace ros;

/* WallStop.cpp
 * Copyright (c) 2018 Ryuichi Ueda <ryuichiueda@gmail.com>
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php */

void WallStop::onSigint(int sig)
{
	std_srvs::Trigger trigger;
	service::call("/motor_off", trigger);
	shutdown();
}

WallStop::WallStop()
{
	service::waitForService("/motor_on");
	service::waitForService("/motor_off");
	signal(SIGINT, WallStop::onSigint);
	std_srvs::Trigger trigger;
	service::call("/motor_on", trigger);
}

void WallStop::callback(const raspimouse_ros_2::LightSensorValues::ConstPtr& msg)
{
	sensor_sum_all = msg->sum_all;
}

void WallStop::run()
{
	geometry_msgs::Twist tw;
	tw.linear.x = 0.2;
	tw.angular.z = 0.0;

	if(sensor_sum_all >= 500)
		tw.linear.x = 0.0;

	pub.publish(tw);
}
