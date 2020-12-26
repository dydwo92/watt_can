/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#ifndef __XARM_HARDWARE_INTERFACE_H__
#define __XARM_HARDWARE_INTERFACE_H__

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
// for mutex

#include <mutex>

#include <sstream>

#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>

#include <net/if.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "innfos.h"

namespace xarm_control
{
	const std::string jnt_state_topic = "joint_states";
	const std::string xarm_state_topic = "xarm_states";

	class XArmHW : public hardware_interface::RobotHW
	{
	public:
		XArmHW(){};
		~XArmHW();
		virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
		virtual void read(const ros::Time& time, const ros::Duration& period);
		virtual void write(const ros::Time& time, const ros::Duration& period);

		/* get current arm status: in the order of state, mode and error_code */
		void get_status(int state_mode_err[3]);
		/* check whether the controller needs to be reset due to error or mode change */
		bool need_reset();

	private:
		int curr_state;
		int curr_mode;
		int curr_err;

		unsigned int dof_;
		std::vector<std::string> jnt_names_;
		std::vector<double> position_cmd_;
		std::vector<float> position_cmd_float_;
		std::vector<double> velocity_cmd_;
		std::vector<double> effort_cmd_;

		std::vector<double> position_fdb_;
		std::vector<double> velocity_fdb_;
		std::vector<double> effort_fdb_;

		double control_rate_;
		bool initial_write_;
		std::mutex mutex_;
		
		//xarm_api::XArmROSClient xarm;
		pthread_t p_thread[2];
		INNFOS_REPLY reply[NUM_MOTOR];

		hardware_interface::JointStateInterface    js_interface_;
	  	hardware_interface::PositionJointInterface pj_interface_;

		ros::Subscriber pos_sub_, vel_sub_, effort_sub_, state_sub_;

		void clientInit(ros::NodeHandle &root_nh);
		//void pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data);
		//void state_fb_cb(const xarm_msgs::RobotMsg::ConstPtr& data);

	};

}

#endif
