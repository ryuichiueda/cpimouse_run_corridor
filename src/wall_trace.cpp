#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/LightSensorValues.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerResponse.h"
#include <fstream>
#include "signal.h"
using namespace ros;

/* wall_trace.cpp
 * Copyright (c) 2018 Ryuichi Ueda <ryuichiueda@gmail.com>
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php */

int sensor_sum_forward = 0;
int sensor_left_side = 0;

void callback(const raspimouse_ros_2::LightSensorValues::ConstPtr& msg)
{
	sensor_sum_forward = msg->sum_forward;
	sensor_left_side = msg->left_side;
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

	if(tw.linear.x < 0.2)
		tw.angular.z = 0.0;
	else if(sensor_left_side < 10)
		tw.angular.z = 0.0;
	else{
		const int target = 50;
	}

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
#!/usr/bin/env python
#encoding: utf-8

#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy,copy,math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from raspimouse_ros_2.msg import LightSensorValues

class WallTrace():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(20)
        data = Twist()

        accel = 0.02
        data.linear.x = 0.0
        data.angular.z = 0
        while not rospy.is_shutdown():
            data.linear.x += accel

            if self.sensor_values.sum_forward > 50:
                data.linear.x = 0.0
            elif data.linear.x <= 0.2:
                data.linear.x = 0.2
            elif data.linear.x >= 0.8:
                data.linear.x = 0.8

            if data.linear.x < 0.2:
                data.angular.z = 0.0
            elif self.sensor_values.left_side < 10:
                data.angular.z = 0.0
            else:
                target = 50
                #1cm近づくと値がだいたい50増える
                error = (target - self.sensor_values.left_side)/50.0
                #1cmあたり3[deg/s]変化をつける
                data.angular.z = error * 3 * math.pi / 180.0

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_trace')

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()

    w = WallTrace()
    w.run()
