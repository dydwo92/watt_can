/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

// #include "xarm_hw.h"
#include "motor_driver_innfos_canopen.h"

int s_can; // Global socketCAN file descriptor

volatile static int interrupted;

void sigint_handler(int sig){
	interrupted = 1;
}

/********************************************************************/
// CAN Thread functions
/********************************************************************/
void* canopen_checkloop(void* d){

	while(ros::ok() && !interrupted){
		INNFOS_timerLoop();
		usleep(1000);
	}

	pthread_exit(NULL);

}

void* canopen_rxloop(void* d){

    struct can_frame frame;

    struct timeval timeout;
    fd_set set;
    int rv;
    int nbytes;

	while(ros::ok() && !interrupted){
	    FD_ZERO(&set);
	    FD_SET(s_can, &set);

	    timeout.tv_sec = 0;
	    timeout.tv_usec = 100000;

	    rv = select(s_can + 1, &set, NULL, NULL, &timeout);
	    if(rv > 0){
	        nbytes = read(s_can, &frame, sizeof(frame));

	        // Add INNFOS response
	        INNFOS_addRxBuffer(frame.can_id, frame.data);
	    }
	}

	pthread_exit(NULL);

}

namespace xarm_control
{
	void XArmHW::clientInit(ros::NodeHandle &root_nh)
	{
		position_cmd_.resize(dof_);
		position_cmd_float_.resize(dof_); // command vector must have 7 dimention!
		position_fdb_.resize(dof_);
		velocity_cmd_.resize(dof_);
		velocity_fdb_.resize(dof_);
		effort_cmd_.resize(dof_);
		effort_fdb_.resize(dof_);

		curr_err = 0;
		curr_state = 0;


		for(unsigned int j=0; j < dof_; j++)
	  	{ 
	  		// Create joint state interface for all joints
	    	js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[j], &position_fdb_[j], &velocity_fdb_[j], &effort_fdb_[j]));

	    	hardware_interface::JointHandle joint_handle;
	    	joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]),&position_cmd_[j]);
	      	pj_interface_.registerHandle(joint_handle);
	  	}

	  	registerInterface(&js_interface_);
	  	registerInterface(&pj_interface_);


	}

	bool XArmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
	{
		std::string hw_ns = robot_hw_nh.getNamespace() + "/";

		std::vector<std::string> jnt_names;
		int xarm_dof = 0;
		double ctrl_rate = 50;

		if(!robot_hw_nh.hasParam("DOF"))
		{
			ROS_ERROR("ROS Parameter xarm_dof not specified!");
			return false;
		}

		if(!robot_hw_nh.hasParam("control_rate"))
		{
			ROS_ERROR("ROS Parameter control_rate not specified!");
			return false;
		}

		/* getParam forbids to change member */
		robot_hw_nh.getParam("DOF", xarm_dof);
		robot_hw_nh.getParam("joint_names", jnt_names);
		robot_hw_nh.getParam("control_rate", ctrl_rate);

		dof_ = xarm_dof;
		jnt_names_ = jnt_names;
		control_rate_ = ctrl_rate;
		initial_write_ = true;

		clientInit(robot_hw_nh);

		// socketCAN variables
		int ret;
		struct sockaddr_can addr;
		struct ifreq ifr;
		struct can_filter rfilter[1];

		// Get socketCAN ----------------------------------------------------->

		// [[ 1.Create socket ]]
		s_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s_can < 0) {
		    ROS_ERROR("socketCAN PF_CAN failed.");
		    return false;
		}

		// [[ 2.Specify can0 device ]]
		strcpy(ifr.ifr_name, "can0");
		ret = ioctl(s_can, SIOCGIFINDEX, &ifr);
		if (ret < 0) {
		    ROS_ERROR("socketCAN ioctl failed.");
		    return false;
		}

		// [[ 3.Bind the socket to can0 ]]
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		ret = bind(s_can, (struct sockaddr *)&addr, sizeof(addr));
		if (ret < 0) {
		    ROS_ERROR("socketCAN bind failed.");
		    return false;
		}

		// [[ 4.Receive all frame ]]
		rfilter[0].can_id = 0x000;
		rfilter[0].can_mask = 0x000;
		setsockopt(s_can, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

		ROS_INFO("can0 socketCAN Enabled.");

		// Get socketCAN <-----------------------------------------------------

	    signal(SIGINT, sigint_handler);

		// Create CAN related threads ---------------------------------------->
	    int thr_id;

	    thr_id = pthread_create(&p_thread[0], NULL, canopen_checkloop, NULL);
	    if(thr_id < 0){
	    	ROS_ERROR("CAN Check loop thread create error.");
	    	return 1;
	    }

	    thr_id = pthread_create(&p_thread[1], NULL, canopen_rxloop, NULL);
	    if(thr_id < 0){
	    	ROS_ERROR("CAN RX loop thread create error.");
	    	return 1;
	    }

	    ROS_INFO("CAN Threads established.");

		// Create CAN related threads <----------------------------------------

	    // [[ Innfos motor to position mode ]]
	    int i;
	    for(i = 0; i < NUM_MOTOR; i++) INNFOS_Init(ID_BASE + i);
	    ROS_INFO("Start operating!!");

		return true;
	}

	XArmHW::~XArmHW()
	{
		int i;
		for(i = 0; i < NUM_MOTOR; i++) INNFOS_deInit(ID_BASE + i);

		int status;
		pthread_join(p_thread[0], (void **)&status);
		pthread_join(p_thread[1], (void **)&status);
		close(s_can);

		ROS_INFO("Bye!");
	}

	void XArmHW::read(const ros::Time& time, const ros::Duration& period)
	{
		// basically the above feedback callback functions have done the job
		int i;
		for(i = 0; i < NUM_MOTOR; i++){
			position_fdb_[i] = reply[i].Position;
		    velocity_fdb_[i] = reply[i].Speed;
			effort_fdb_[i] = 0;
		}

	}

	void XArmHW::write(const ros::Time& time, const ros::Duration& period)
	{
		if(need_reset())
		{
			std::lock_guard<std::mutex> locker(mutex_);
			for(int k=0; k<dof_; k++)
			{
				position_cmd_float_[k] = (float)position_fdb_[k];
			}
			return;
		}

		for(int k=0; k<dof_; k++)
		{
			// make sure no abnormal command will be written into joints, check if cmd velocity > [180 deg/sec * (1+10%)]
			if(fabs(position_cmd_float_[k]-(float)position_cmd_[k])*control_rate_ > 3.14*1.25  && !initial_write_)
			{
				ROS_WARN("joint %d abnormal command! previous: %f, this: %f\n", k+1, position_cmd_float_[k], (float)position_cmd_[k]);
				// return;
			}

			position_cmd_float_[k] = (float)position_cmd_[k];
		}

		//xarm.setServoJ(position_cmd_float_);
		int i;
		for(i = 0; i < NUM_MOTOR; i++){
			INNFOS_posCmd(reply + i, ID_BASE + i, position_cmd_float_[i], 100);
		}

		initial_write_ = false;
	}

	void XArmHW::get_status(int state_mode_err[3])
	{
		state_mode_err[0] = curr_state;
		state_mode_err[1] = curr_mode;
		state_mode_err[2] = curr_err;
	}

	bool XArmHW::need_reset()
	{
		if(curr_state==4 || curr_state==5 || curr_err)
			return true;
		else
			return false;
	}
}
