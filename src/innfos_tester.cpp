#include "ros/ros.h"
#include "watt_can/Innfosreq.h"
#include "watt_can/Innfos6res.h"

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

#define ID_BASE	0xC0
#define NUM_MOTOR	6

int s_can; // Global socketCAN file descriptor

static int interrupted;

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

/********************************************************************/
// ROS subscriber callback
/********************************************************************/
static ros::Publisher *pub_instance;
static watt_can::Innfos6res pub_data;
static INNFOS_REPLY reply[NUM_MOTOR];
void chatterCallback(const watt_can::Innfosreq::ConstPtr& msg)
{
	float temp;
	int32_t temp32;
	int i;

	for(i = 0; i < NUM_MOTOR; i++){
		INNFOS_posCmd(reply + i, ID_BASE + i, msg->position[i], 200);
		pub_data.m[i].Speed = reply[i].Speed;
		pub_data.m[i].Position = reply[i].Position;
		pub_data.m[i].Current = reply[i].Current;
		pub_data.m[i].Voltage = reply[i].Voltage;
		pub_data.m[i].m_temp = reply[i].m_temp;
		pub_data.m[i].d_temp = reply[i].d_temp;
	}

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

	ros::Subscriber sub = n.subscribe("request_innfos", 1000, chatterCallback);
	ros::Publisher pub = n.advertise<watt_can::Innfos6res>("response_innfos", 1000);
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

    // [[ Innfos motor to position mode ]]
    int i;
    for(i = 0; i < NUM_MOTOR; i++) INNFOS_Init(ID_BASE + i);


    // Initial CAN Batch <-------------------------------------------------

    ROS_INFO("Start operating!!");

	while(ros::ok() && !interrupted){
		ros::spinOnce();
	}

	for(i = 0; i < NUM_MOTOR; i++) INNFOS_deInit(ID_BASE + i);

	int status;
	pthread_join(p_thread[0], (void **)&status);
	pthread_join(p_thread[1], (void **)&status);
	close(s_can);

	ROS_INFO("Bye!");

	return 0;

}
