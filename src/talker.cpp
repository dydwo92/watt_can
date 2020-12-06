#include "ros/ros.h"
#include "watt_can/Wattreq.h"
#include "watt_can/Wattres.h"

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
#include "gyems.h"

#define M1_ID	0x02
#define M2_ID	0x01
#define M3_ID	0x02

int s_can; // Global socketCAN file descriptor

static GYEMS_REPLY gyems_reply;
static int interrupted;

void sigint_handler(int sig){
	interrupted = 1;
}

/********************************************************************/
// CAN Thread functions
/********************************************************************/
void* canopen_checkloop(void* d){

	while(ros::ok() && !interrupted){
		CANOpen_timerLoop();
		GYEMS_timerLoop();
		usleep(100);
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

	        // Check frame is CANOpen or GYEMS
	        if((frame.can_id & 0x140) == 0x140) GYEMS_addRxBuffer(frame.can_id, frame.data);
 	        else CANOpen_addRxBuffer(frame.can_id, frame.data);
	    }
	}

	pthread_exit(NULL);

}

/********************************************************************/
// ROS subscriber callback
/********************************************************************/
static ros::Publisher *pub_instance;
static watt_can::Wattres pub_data;

void chatterCallback(const watt_can::Wattreq::ConstPtr& msg)
{
	// M1 Communication ------------------------------------->
	int32_t m1_data;
	uint8_t m1_rxlen;
	CANOpen_writeOD_int32(M1_ID, 0x60FF, 0x00, msg->m1, 10);
	CANOpen_readOD(M1_ID, 0x606C, 0x00, (uint8_t*)&m1_data, &m1_rxlen, 10);
	pub_data.m1 = m1_data / 10;
	// M1 Communication <-------------------------------------

	// M2 Communication ------------------------------------->
	bool m2_response = false;
	switch(msg->m2.msgType){
	case 1 :
		m2_response = GYEMS_posCmd1(
				&gyems_reply,
				M2_ID,
				msg->m2.angleControl,
				10);
		break;

	case 2 :
		m2_response = GYEMS_posCmd2(
				&gyems_reply,
				M2_ID,
				msg->m2.maxSpeed,
				msg->m2.angleControl,
				10);
		break;

	case 3:
		m2_response = GYEMS_posCmd3(
				&gyems_reply,
				M2_ID,
				msg->m2.spinDirection,
				msg->m2.angleControl,
				10);
		 break;

	case 4 :
		m2_response = GYEMS_posCmd4(
				&gyems_reply,
				M2_ID,
				msg->m2.spinDirection,
				msg->m2.maxSpeed,
				msg->m2.angleControl,
				10);
		break;
	}
	if(m2_response){
		pub_data.m2.temperature = gyems_reply.temperature;
		pub_data.m2.iq = gyems_reply.iq;
		pub_data.m2.speed = gyems_reply.speed;
		pub_data.m2.encoder = gyems_reply.encoder;

	}
	// M2 Communication <-------------------------------------

	// M3 Communication ------------------------------------->
	bool m3_response = false;
	switch(msg->m3.msgType){
	case 1 :
		m3_response = GYEMS_posCmd1(
				&gyems_reply,
				M3_ID,
				msg->m3.angleControl,
				100);
		break;

	case 2 :
		m3_response = GYEMS_posCmd2(
				&gyems_reply,
				M3_ID,
				msg->m3.maxSpeed,
				msg->m3.angleControl,
				100);
		break;

	case 3:
		m3_response = GYEMS_posCmd3(
				&gyems_reply,
				M3_ID,
				msg->m3.spinDirection,
				msg->m3.angleControl,
				100);
		 break;

	case 4 :
		m3_response = GYEMS_posCmd4(
				&gyems_reply,
				M3_ID,
				msg->m3.spinDirection,
				msg->m3.maxSpeed,
				msg->m3.angleControl,
				100);
		break;
	}

	if(m3_response){
		pub_data.m3.temperature = gyems_reply.temperature;
		pub_data.m3.iq = gyems_reply.iq;
		pub_data.m3.speed = gyems_reply.speed;
		pub_data.m3.encoder = gyems_reply.encoder;

	}
	// M3 Communication <-------------------------------------

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

	ros::Subscriber sub = n.subscribe("request", 1000, chatterCallback);
	ros::Publisher pub = n.advertise<watt_can::Wattres>("response", 1000);
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

    // [[ ZLAC8015 Driver at 0x02 (CANOpen Protocol)
    CANOpen_writeOD_uint16(0x02, 0x6040, 0x00, 0x0000, 1000);
    CANOpen_writeOD_uint32(0x02, 0x6083, 0x00, 0x1, 1000);
    CANOpen_writeOD_uint32(0x02, 0x6084, 0x00, 0x1, 1000);
    CANOpen_writeOD_int8(0x02, 0x6060, 0x00, 0x03, 1000);
    sleep(1);
    CANOpen_writeOD_uint16(0x02, 0x6040, 0x00, 0x0006, 1000);
    CANOpen_writeOD_uint16(0x02, 0x6040, 0x00, 0x0007, 1000);
    CANOpen_writeOD_uint16(0x02, 0x6040, 0x00, 0x000F, 1000);


    // Initial CAN Batch <-------------------------------------------------

    ROS_INFO("Start operating!!");

	while(ros::ok() && !interrupted){
		ros::spinOnce();
	}

    CANOpen_writeOD_uint16(0x02, 0x6040, 0x00, 0x0000, 1000);

	int status;
	pthread_join(p_thread[0], (void **)&status);
	pthread_join(p_thread[1], (void **)&status);
	close(s_can);

	ROS_INFO("Bye!");

	return 0;

}
