#include "motor_driver_zlac8015_canopen.h"

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
		CANOpen_timerLoop();
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

	        CANOpen_addRxBuffer(frame.can_id, frame.data);
	    }
	}

	pthread_exit(NULL);

}

/********************************************************************/
// ROS Control functions
/********************************************************************/
MotorDriverZLAC8015CANOpenInterface::MotorDriverZLAC8015CANOpenInterface()
{
    last_encoder_value_.resize(2, 0);
}

MotorDriverZLAC8015CANOpenInterface::~MotorDriverZLAC8015CANOpenInterface()
{
    stop_driver(0);

    ROS_INFO("Stop zalc8015 driver");

    CANOpen_writeOD_uint16(M1_ID, 0x6040, 0x00, 0x0000, 0);
    CANOpen_writeOD_uint16(M2_ID, 0x6040, 0x00, 0x0000, 0);

	int status;
	pthread_join(p_thread[0], (void **)&status);
	pthread_join(p_thread[1], (void **)&status);
	close(s_can);

	ROS_INFO("Bye!");
}

bool MotorDriverZLAC8015CANOpenInterface::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{

    // Get socketCAN ----------------------------------------------------->

	// [[ 1.Create socket ]]
	s_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s_can < 0) {
	    ROS_ERROR("socketCAN PF_CAN failed.");
	    return 1;
	}

	// [[ 2.Specify can0 device ]]
	strcpy(ifr.ifr_name, "can0");
	ret = ioctl(s_can, SIOCGIFINDEX, &ifr);
	if (ret < 0) {
	    ROS_ERROR("socketCAN ioctl failed.");
	    return 1;
	}

	// [[ 3.Bind the socket to can0 ]]
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	ret = bind(s_can, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
	    ROS_ERROR("socketCAN bind failed.");
	    return 1;
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

    // [[ ZLAC8015 0x01 PDO Settings ]]
    // RPDO1 Setting
    CANOpen_writeOD_uint32(M1_ID, 0x1400, 0x01, 0x80000200 | M1_ID, 1000);
    CANOpen_writeOD_uint8(M1_ID, 0x1400, 0x02, 0x01, 1000);
    CANOpen_writeOD_uint8(M1_ID, 0x1600, 0x00, 0, 1000);
    CANOpen_writeOD_uint32(M1_ID, 0x1600, 0x01, 0x60FF0020, 1000);
    CANOpen_writeOD_uint8(M1_ID, 0x1600, 0x00, 1, 1000);
    CANOpen_writeOD_uint32(M1_ID, 0x1400, 0x01, 0x200 | M1_ID, 1000);

    CANOpen_mappingPDO_init(&m1_sendPDO);
    CANOpen_mappingPDO_int32(&m1_sendPDO, &m1_speed_input);

    // TPDO1 Setting
    CANOpen_writeOD_uint32(M1_ID, 0x1800, 0x01, 0x80000180 | M1_ID, 1000);
    CANOpen_writeOD_uint8(M1_ID, 0x1800, 0x02, 0x01, 1000);
    CANOpen_writeOD_uint8(M1_ID, 0x1A00, 0x00, 0, 1000);
    CANOpen_writeOD_uint32(M1_ID, 0x1A00, 0x01, 0x60640020, 1000);
    CANOpen_writeOD_uint32(M1_ID, 0x1A00, 0x02, 0x606C0020, 1000);
    CANOpen_writeOD_uint8(M1_ID, 0x1A00, 0x00, 2, 1000);
    CANOpen_writeOD_uint32(M1_ID, 0x1800, 0x01, 0x180 | M1_ID, 1000);

    CANOpen_mappingPDO_init(&m1_readPDO);
    CANOpen_mappingPDO_int32(&m1_readPDO, &m1_position_output);
    CANOpen_mappingPDO_int32(&m1_readPDO, &m1_speed_output);

    // [[ ZLAC8015 0x02 PDO Settings ]]
    // RPDO1 Setting
    CANOpen_writeOD_uint32(M2_ID, 0x1400, 0x01, 0x80000200 | M2_ID, 1000);
    CANOpen_writeOD_uint8(M2_ID, 0x1400, 0x02, 0x01, 1000);
    CANOpen_writeOD_uint8(M2_ID, 0x1600, 0x00, 0, 1000);
    CANOpen_writeOD_uint32(M2_ID, 0x1600, 0x01, 0x60FF0020, 1000);
    CANOpen_writeOD_uint8(M2_ID, 0x1600, 0x00, 1, 1000);
    CANOpen_writeOD_uint32(M2_ID, 0x1400, 0x01, 0x200 | M2_ID, 1000);

    CANOpen_mappingPDO_init(&m2_sendPDO);
    CANOpen_mappingPDO_int32(&m2_sendPDO, &m2_speed_input);

    // TPDO1 Setting
    CANOpen_writeOD_uint32(M2_ID, 0x1800, 0x01, 0x80000180 | M2_ID, 1000);
    CANOpen_writeOD_uint8(M2_ID, 0x1800, 0x02, 0x01, 1000);
    CANOpen_writeOD_uint8(M2_ID, 0x1A00, 0x00, 0, 1000);
    CANOpen_writeOD_uint32(M2_ID, 0x1A00, 0x01, 0x60640020, 1000);
    CANOpen_writeOD_uint32(M2_ID, 0x1A00, 0x02, 0x606C0020, 1000);
    CANOpen_writeOD_uint8(M2_ID, 0x1A00, 0x00, 2, 1000);
    CANOpen_writeOD_uint32(M2_ID, 0x1800, 0x01, 0x180 | M2_ID, 1000);

    CANOpen_mappingPDO_init(&m2_readPDO);
    CANOpen_mappingPDO_int32(&m2_readPDO, &m2_position_output);
    CANOpen_mappingPDO_int32(&m2_readPDO, &m2_speed_output);

    // [[ Start driver ]]
    CANOpen_writeOD_uint16(M1_ID, 0x6040, 0x00, 0x0000, 1000);
    CANOpen_writeOD_uint32(M1_ID, 0x6083, 0x00, 0x1, 1000);
    CANOpen_writeOD_uint32(M1_ID, 0x6084, 0x00, 0x1, 1000);
    CANOpen_writeOD_int8(M1_ID, 0x6060, 0x00, 0x03, 1000);

    CANOpen_writeOD_uint16(M2_ID, 0x6040, 0x00, 0x0000, 1000);
    CANOpen_writeOD_uint32(M2_ID, 0x6083, 0x00, 0x1, 1000);
    CANOpen_writeOD_uint32(M2_ID, 0x6084, 0x00, 0x1, 1000);
    CANOpen_writeOD_int8(M2_ID, 0x6060, 0x00, 0x03, 1000);

    sleep(1);
    CANOpen_writeOD_uint16(M1_ID, 0x6040, 0x00, 0x0006, 1000);
    CANOpen_writeOD_uint16(M1_ID, 0x6040, 0x00, 0x0007, 1000);
    CANOpen_writeOD_uint16(M1_ID, 0x6040, 0x00, 0x000F, 1000);

    CANOpen_writeOD_uint16(M2_ID, 0x6040, 0x00, 0x0006, 1000);
    CANOpen_writeOD_uint16(M2_ID, 0x6040, 0x00, 0x0007, 1000);
    CANOpen_writeOD_uint16(M2_ID, 0x6040, 0x00, 0x000F, 1000);

    CANOpen_NMT(CO_OP);
    sleep(1);

    // hardware_interface
    joint_cmd_.resize(2);
    for(int i = 0; i < joint_cmd_.size(); i++) {
        joint_cmd_[i] = 0.0;
    }
    joint_pos_.resize(2, 0);
    joint_vel_.resize(2, 0);
    joint_eff_.resize(2, 0);

    hardware_interface::JointStateHandle state_handle_l_wheel("l_wheel_joint", &joint_pos_[0], &joint_vel_[0], &joint_eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_l_wheel);
    hardware_interface::JointStateHandle state_handle_r_wheel("r_wheel_joint", &joint_pos_[1], &joint_vel_[1], &joint_eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_r_wheel);

    registerInterface(&jnt_state_interface_);

    // joint_velocity_interface
    hardware_interface::JointHandle vel_handle_l_wheel(jnt_state_interface_.getHandle("l_wheel_joint"), &joint_cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_l_wheel);
    hardware_interface::JointHandle vel_handle_r_wheel(jnt_state_interface_.getHandle("r_wheel_joint"), &joint_cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_r_wheel);

    registerInterface(&jnt_vel_interface_);

    e_stop_active_ = false;
    last_e_stop_active_ = false;
    sub_e_stop_ = nh.subscribe("activate_e_stop", 1, &MotorDriverZLAC8015CANOpenInterface::callback_activate_e_stop, this);

    // ROS
    srv_clear_alarm_ = nh.advertiseService("clear_motor_alarm", &MotorDriverZLAC8015CANOpenInterface::handle_clear_alarm, this);

    ROS_INFO("[%s] initialized successfully.", ros::this_node::getName().c_str());
    return true;
}

void MotorDriverZLAC8015CANOpenInterface::callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active)
{
    e_stop_active_ = e_stop_active->data;
}

void MotorDriverZLAC8015CANOpenInterface::read(const ros::Time& time, const ros::Duration& period)
{
    boost::mutex::scoped_lock scoped_lock(lock);

    int32_t current_enc[2] = {0, 0};

    // Communicate
    CANOpen_sendSync();
    CANOpen_readPDO(M1_ID, 1, &m1_readPDO, 10);
    CANOpen_readPDO(M2_ID, 1, &m2_readPDO, 10);


    joint_eff_[0] = 0;
    joint_eff_[1] = 0;

    joint_vel_[0] = m1_speed_output / 10.0 / 60.0 * 2.0 * M_PI;
    joint_vel_[1] = m2_speed_output / 10.0 / 60.0 * 2.0 * M_PI;

    joint_pos_[0] = m1_position_output / 4096.0 * 2.0 * M_PI;
    joint_pos_[1] = -m2_position_output / 4096.0 * 2.0 * M_PI;

    printf("%d\t%d\t%d\t%d\n", m1_speed_output, m2_speed_output, m1_position_output, m2_position_output);
    
    /*

    current_enc[0] = m1_position_output;
    current_enc[1] = m2_position_output;

    joint_eff_[0] = 0.0; //u_current[0] * -1.0;
    joint_eff_[1] = 0.0; //u_current[1] * -1.0;

    joint_vel_[0] = (current_enc[0] - last_encoder_value_[0]) / 4096.0 * (1.0 / period.toSec()) * (2.0 * M_PI);
    joint_vel_[1] = (current_enc[1] - last_encoder_value_[1]) / 4096.0 * (1.0 / period.toSec()) * (2.0 * M_PI) * 1.0;

    joint_pos_[0] += (current_enc[0] - last_encoder_value_[0]) / 4096.0 * (2.0 * M_PI);
    joint_pos_[1] += (current_enc[1] - last_encoder_value_[1]) / 4096.0 * (2.0 * M_PI) * -1.0;

    last_encoder_value_[0] = current_enc[0];
    last_encoder_value_[1] = current_enc[1];
    */
}

void MotorDriverZLAC8015CANOpenInterface::write(const ros::Time& time, const ros::Duration& period)
{
    if(is_has_error_)
        return;

    // When e_stop_active_ is true (E-STOP Button is pressed, all joint command for velocity is set 0)
    joint_cmd_[0] = e_stop_active_ ? 0 : joint_cmd_[0];
    joint_cmd_[1] = e_stop_active_ ? 0 : joint_cmd_[1];

    int32_t l_vel = (int32_t)(joint_cmd_[0] * 60.0 / (2.0 * M_PI));
    int32_t r_vel = (int32_t)(joint_cmd_[1] * 60.0 / (2.0 * M_PI)) * -1.0;

    boost::mutex::scoped_lock scoped_lock(lock);

    m1_speed_input = l_vel;
    m2_speed_input = r_vel;

    CANOpen_sendPDO(M1_ID, 1, &m1_sendPDO);
    CANOpen_sendPDO(M2_ID, 1, &m2_sendPDO);
}

bool MotorDriverZLAC8015CANOpenInterface::handle_clear_alarm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    return true;
}


///
bool MotorDriverZLAC8015CANOpenInterface::reset_driver(int id)
{
    CANOpen_NMT(CO_RESET);
    return true;
}

bool MotorDriverZLAC8015CANOpenInterface::start_driver(int id)
{
    CANOpen_NMT(CO_OP);
    return true;
}

bool MotorDriverZLAC8015CANOpenInterface::stop_driver(int id)
{
    CANOpen_NMT(CO_STOPPED);
    return  true;
}


///

bool MotorDriverZLAC8015CANOpenInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                ROS_ERROR_STREAM("Bad interface: " << res_it->hardware_interface);
                std::cout << res_it->hardware_interface;
                return false;
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    ROS_ERROR_STREAM("Bad resource: " << (*ctrl_res));
                    std::cout << (*ctrl_res);
                    return false;
                }
            }
        }
    }
    return true;
}

void MotorDriverZLAC8015CANOpenInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                throw hardware_interface::HardwareInterfaceException("Hardware_interface " + res_it->hardware_interface + " is not registered");
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    throw hardware_interface::HardwareInterfaceException("Resource " + *ctrl_res + " is not registered");
                }
            }
        }
    }
}
