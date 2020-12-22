#include "ros/ros.h"
#include "watt_can/Zlactestreq.h"
#include "watt_can/Zlactestres.h"

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

CO_PDOStruct m1_sendPDO, m1_readPDO;
int32_t m1_speed_input;
int32_t m1_speed_output;
int32_t m1_position_output;

CO_PDOStruct m2_sendPDO, m2_readPDO;
int32_t m2_speed_input;
int32_t m2_speed_output;
int32_t m2_position_output;

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
// ROS subscriber callback
/********************************************************************/
static ros::Publisher *pub_instance;
static watt_can::Zlactestres pub_data;

void chatterCallback(const watt_can::Zlactestreq::ConstPtr& msg)
{

   m1_speed_input = msg->m1_speed;;
   m2_speed_input = msg->m2_speed;

   // Communicate
   CANOpen_sendPDO(M1_ID, 1, &m1_sendPDO);
   CANOpen_sendPDO(M2_ID, 1, &m2_sendPDO);
   CANOpen_sendSync();
   CANOpen_readPDO(M1_ID, 1, &m1_readPDO, 10);
   CANOpen_readPDO(M2_ID, 1, &m2_readPDO, 10);

   pub_data.m1_speed = m1_speed_output; 
   pub_data.m2_speed = m2_speed_output;
   pub_data.m1_position = m1_position_output;
   pub_data.m2_position = m2_position_output;

	if(pub_instance){
		pub_instance->publish(pub_data);
	}
}

/********************************************************************/
// Main loop
/********************************************************************/

// Before run this program, enalbe can0 socketCAN
// $ sudo ip link set can0 type can bitrate 1000000
// $ sudo ifconfig can0 up

int main(int argc, char **argv){

	// socketCAN variables
	int ret;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_filter rfilter[1];

	// ROS variables
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
    ros::Rate r(10);

	ros::Subscriber sub = n.subscribe("request", 1000, chatterCallback);
	ros::Publisher pub = n.advertise<watt_can::Zlactestres>("response", 1000);
	pub_instance = &pub;

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

    pthread_t p_thread[2];
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

    // Initial CAN Batch ------------------------------------------------->

    // [[ NMT Reset ]]
    ROS_INFO("CANOpen NMT Reset.");
    CANOpen_NMT(CO_RESET);
    sleep(1);
    

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

    // Initial CAN Batch <-------------------------------------------------

    ROS_INFO("Start operating!!");

	while(ros::ok() && !interrupted){
		ros::spinOnce();
        r.sleep();
	}


    CANOpen_writeOD_uint16(M1_ID, 0x6040, 0x00, 0x0000, 0);
    CANOpen_writeOD_uint16(M2_ID, 0x6040, 0x00, 0x0000, 0);

	int status;
	pthread_join(p_thread[0], (void **)&status);
	pthread_join(p_thread[1], (void **)&status);
	close(s_can);

	ROS_INFO("Bye!");

	return 0;

}
