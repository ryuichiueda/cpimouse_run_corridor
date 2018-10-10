#include "WallStop.h"

/* wall_stop.cpp
 * Copyright (c) 2018 Ryuichi Ueda <ryuichiueda@gmail.com>
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php */

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

