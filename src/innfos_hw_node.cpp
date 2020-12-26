
/* Copyright 2020 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

// #include "xarm_hw.h"
#include <motor_driver_innfos_canopen.h>

int main(int argc, char**argv)
{
	ros::init(argc, argv, "innfos_driver");

	ros::NodeHandle nh;

	double ctrl_rate = 50;
	nh.setParam("control_rate", ctrl_rate);
	ros::Rate r(ctrl_rate);
	
	ros::Duration(1.0).sleep();

	xarm_control::XArmHW xarm_hw;
	if(!xarm_hw.init(nh, nh)) exit(-1);

	controller_manager::ControllerManager cm(&xarm_hw, nh);

  	ros::AsyncSpinner spinner(4);
	spinner.start();

	// IMPORTANT: DO NOT REMOVE THIS DELAY !!!
	/* Wait for correct initial position to be updated to ros_controller */
	ros::Duration(2.0).sleep();

	ros::Time ts = ros::Time::now();
	while (ros::ok())
	{	
	   ros::Duration elapsed = ros::Time::now() - ts;
	   ts = ros::Time::now();
	   xarm_hw.read(ts, elapsed);
	   cm.update(ts, elapsed);
	   xarm_hw.write(ts, elapsed);
	   r.sleep();
	}
	spinner.stop();
	return 0;
}
