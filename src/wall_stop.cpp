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


void callback(const raspimouse_ros_2::LightSensorValues::ConstPtr& msg)
{

}

void onSigint(int sig)
{
	std_srvs::Trigger trigger;
	service::call("/motor_off", trigger);
	shutdown();
}

void run(void)
{
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

	unsigned int c = 0;
	while(ok()){
		run();
		/*
		std::ifstream ifs("/dev/rtlightsensor0");
		ifs >> msg.right_forward >> msg.right_side
			>> msg.left_side >> msg.left_forward;

		msg.sum_forward = msg.left_forward + msg.right_forward;
		msg.sum_all = msg.sum_forward + msg.left_side + msg.right_side;

		pub.publish(msg);
		*/
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}

/*


class WallTrace():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(10)
        data = Twist()

        while not rospy.is_shutdown():
            data.linear.x = 0.2
            data.angular.z = 0
            if self.sensor_values.sum_all >= 500:
                data.linear.x = 0
                data.angular.z = 0

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_trace')

    w = WallTrace()
    w.run()
    */
