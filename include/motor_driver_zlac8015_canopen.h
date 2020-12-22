#ifndef MOTOR_DRIVER_ZLAC8015_CANOPEN_H_
#define MOTOR_DRIVER_ZLAC8015_CANOPEN_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>


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

#include "CANOpen.h"

#define M1_ID	0x01
#define M2_ID	0x02

extern int s_can; // Global socketCAN file descriptor

class MotorDriverZLAC8015CANOpenInterface: public hardware_interface::RobotHW
{
    public:
        MotorDriverZLAC8015CANOpenInterface();
        ~MotorDriverZLAC8015CANOpenInterface();

    public:
        virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        virtual void read(const ros::Time& time, const ros::Duration& period);
        virtual void write(const ros::Time& time, const ros::Duration& period);
        virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
        virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    private:
        void callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active);
        bool handle_clear_alarm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // Driver
        bool reset_driver(int id);
        bool start_driver(int id);
        bool stop_driver(int id);

    private:


        // CAN & Interface variables
        int ret;
        struct sockaddr_can addr;
        struct ifreq ifr;
        struct can_filter rfilter[1];

        CO_PDOStruct m1_sendPDO, m1_readPDO;
        int32_t m1_speed_input;
        int32_t m1_speed_output;
        int32_t m1_position_output;

        CO_PDOStruct m2_sendPDO, m2_readPDO;
        int32_t m2_speed_input;
        int32_t m2_speed_output;
        int32_t m2_position_output;

        pthread_t p_thread[2];

        hardware_interface::JointStateInterface jnt_state_interface_;
        hardware_interface::PositionJointInterface jnt_pos_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        hardware_interface::EffortJointInterface jnt_eff_interface_;

        std::vector<double> joint_cmd_;
        std::vector<double> last_joint_cmd_;
        std::vector<double> joint_pos_;
        std::vector<double> joint_vel_;
        std::vector<double> joint_eff_;

        std::vector<int32_t> last_encoder_value_;

        bool e_stop_active_;
        bool last_e_stop_active_;
        bool is_has_error_;
        ros::Subscriber sub_e_stop_;

        boost::mutex lock;

        ros::ServiceServer srv_clear_alarm_;
};

#endif //MOTOR_DRIVER_ZLAC8015_CANOPEN_H_