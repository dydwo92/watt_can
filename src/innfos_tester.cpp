#include "ros/ros.h"
#include "watt_can/Innfosreq.h"
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


#define M1_ID	0x02
#define M2_ID	0x5A

#define IQ24			16777216.0f		//2^24

int s_can; // Global socketCAN file descriptor
uint8_t can_tx[8];

static int interrupted;

void sigint_handler(int sig){
	interrupted = 1;
}

void CAN_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len){

  /* USER CODE BEGIN 1 */

  static struct can_frame frame;

  static struct timeval timeout;
  static fd_set set;
  static int rv;

  frame.can_id = cobID;
  frame.can_dlc = len;
  memcpy(frame.data, data, len);

  FD_ZERO(&set);
  FD_SET(s_can, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 100000;

  rv = select(s_can + 1, NULL, &set, NULL, &timeout);
  if(rv >= 0 && FD_ISSET(s_can, &set)){
	  write(s_can, &frame, sizeof(frame));
  }

  /* USER CODE END 1 */

}

/********************************************************************/
// CAN Thread functions
/********************************************************************/
void* canopen_checkloop(void* d){

	while(ros::ok() && !interrupted){
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

	        // Check frame is CANOpen or GYEMS
	        //if((frame.can_id & 0x140) == 0x140) GYEMS_addRxBuffer(frame.can_id, frame.data);
 	        //else CANOpen_addRxBuffer(frame.can_id, frame.data);
	    }
	}

	pthread_exit(NULL);

}

/********************************************************************/
// ROS subscriber callback
/********************************************************************/
static ros::Publisher *pub_instance;
static watt_can::Wattres pub_data;

void chatterCallback(const watt_can::Innfosreq::ConstPtr& msg)
{
	float temp;
	int32_t temp32;

	temp = msg->m1_position * IQ24;
	//temp /= 6000.0f;
	temp /= 360.0f;
	temp32 = (int32_t)temp;
	can_tx[0] = 0x0A;
	can_tx[1] = (uint8_t)(temp32 >> 24);
	can_tx[2] = (uint8_t)(temp32 >> 16);
	can_tx[3] = (uint8_t)(temp32 >> 8);
	can_tx[4] = (uint8_t)temp32;
	CAN_sendFrame(M1_ID, can_tx, 5);


	temp = msg->m2_position * IQ24;
	temp /= 360.0f;
	//temp /= 6000.0f;
	temp32 = (int32_t)temp;
	printf("%d\n", temp32);
	can_tx[0] = 0x0A;
	can_tx[1] = (uint8_t)(temp32 >> 24);
	can_tx[2] = (uint8_t)(temp32 >> 16);
	can_tx[3] = (uint8_t)(temp32 >> 8);
	can_tx[4] = (uint8_t)temp32;
	CAN_sendFrame(M2_ID, can_tx, 5);

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

    // [[ Innfos motor to position mode ]]
    can_tx[0] = 0x2A; can_tx[1] = 0x01; CAN_sendFrame(M1_ID, can_tx, 2); // SCA enable
    sleep(1);
    can_tx[0] = 0x07; can_tx[1] = 0x06; CAN_sendFrame(M1_ID, can_tx, 2); // Select usage mode [ position loop ]
    sleep(1);
    can_tx[0] = 0x2A; can_tx[1] = 0x01; CAN_sendFrame(M2_ID, can_tx, 2); // SCA enable
    sleep(1);
    can_tx[0] = 0x07; can_tx[1] = 0x06; CAN_sendFrame(M2_ID, can_tx, 2); // Select usage mode [ position loop ]
    sleep(1);


    // Initial CAN Batch <-------------------------------------------------

    ROS_INFO("Start operating!!");

	while(ros::ok() && !interrupted){
		ros::spinOnce();
	}

	can_tx[0] = 0x2A; can_tx[1] = 0x00; CAN_sendFrame(M1_ID, can_tx, 2); // SCA disable
	can_tx[0] = 0x2A; can_tx[1] = 0x00; CAN_sendFrame(M2_ID, can_tx, 2); // SCA disable

	int status;
	pthread_join(p_thread[0], (void **)&status);
	pthread_join(p_thread[1], (void **)&status);
	close(s_can);

	ROS_INFO("Bye!");

	return 0;

}
