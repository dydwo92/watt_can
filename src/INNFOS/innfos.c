#include "innfos.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <linux/can.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

extern int s_can;

typedef struct INNFOS_BUFFER{
  uint8_t id;
  uint8_t data[8];
  uint8_t valid;
  uint16_t timeout;
}INNFOS_BUFFER;
static INNFOS_BUFFER rxBuffer[BUFLEN];

static uint8_t innfos_data[8];

volatile static uint16_t rx_timeout;

static uint8_t lock_buff = 0;
static struct INNFOS_BUFF* buff_head = NULL;
static struct INNFOS_BUFF* buff_tail = NULL;

////////////////////////////////////////////////////////////////////
// CAN Send/Get function
////////////////////////////////////////////////////////////////////
void INNFOS_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len){

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

}

void INNFOS_addRxBuffer(uint8_t id, uint8_t* data){

  uint16_t i;
  for(i = 0; i < BUFLEN; i++){
    if(rxBuffer[i].valid == 0){
      rxBuffer[i].timeout = RX_TIMEOUT;
      rxBuffer[i].id = id;
      memcpy(rxBuffer[i].data, data, 8);
      rxBuffer[i].valid = 1;
      return;
    }
  }
  
}

void INNFOS_timerLoop(){ // Should be call every 1ms
  
  uint16_t i;
  if(rx_timeout != 0) rx_timeout --;
  for(i = 0; i < BUFLEN; i++){
    if(rxBuffer[i].valid == 1){
      if(rxBuffer[i].timeout != 0) rxBuffer[i].timeout --;
      if(rxBuffer[i].timeout == 0) rxBuffer[i].valid = 0;
    }
  }
  
}

bool INNFOS_parseResponse(INNFOS_REPLY *reply, uint8_t id){

	int i;

	int32_t temp32;
	int16_t temp16;

	bool get_CVP = false;
	bool get_Vbat = false;
	bool get_Mtemp = false;
	bool get_Dtemp = false;

	i = 0;
	while(rx_timeout){

		if(i >= BUFLEN) return false;

		if(rxBuffer[i].valid == 0){
			i ++;
			continue;
		}
		
		if(rxBuffer[i].id != id){
			i ++;
			continue;
		}

		switch(rxBuffer[i].data[0]){
		case 0x94 : // Get CVP
			if(!get_CVP) get_CVP = true;

			temp32  = ((int32_t)rxBuffer[i].data[1])<<24;
		    temp32 |= ((int32_t)rxBuffer[i].data[2])<<16;
		    temp32 |= ((int32_t)rxBuffer[i].data[3])<<8;
		    reply->Position = (float)temp32 / IQ24;
		    reply->Position /= 36.0f;
		    reply->Position *= M_PI2;

		    temp32  = ((int32_t)rxBuffer[i].data[4])<<24;
	   	    temp32 |= ((int32_t)rxBuffer[i].data[5])<<16;
		    reply->Speed = (float)temp32 / IQ30 * Velocity_Max; // RPM
		    reply->Speed /= 60.0f;
		    reply->Speed *= M_PI2;

		    temp32  = ((int32_t)rxBuffer[i].data[6])<<24;
		    temp32 |= ((int32_t)rxBuffer[i].data[7])<<16;
		    reply->Current  = (float)temp32 / IQ30;

		    rxBuffer[i].valid = 0;
			break;

		case 0x45 : // Get Voltage
			if(!get_Vbat) get_Vbat = true;

			temp16  = ((int16_t)rxBuffer[i].data[1])<<8;
			temp16 |= ((int16_t)rxBuffer[i].data[2])<<0;
			reply->Voltage = (float)temp16 / IQ10;

		    rxBuffer[i].valid = 0;
			break;

		case 0x5F : // Get Motor temperature
			if(!get_Mtemp) get_Mtemp = true;

			temp16  = ((int16_t)rxBuffer[i].data[1])<<8;
			temp16 |= ((int16_t)rxBuffer[i].data[2])<<0;
			reply->m_temp = (float)temp16 / IQ8;

		    rxBuffer[i].valid = 0;
			break;

		case 0x60 : // Get Driver temperature
			if(!get_Dtemp) get_Dtemp = true;

			temp16  = ((int16_t)rxBuffer[i].data[1])<<8;
			temp16 |= ((int16_t)rxBuffer[i].data[2])<<0;
			reply->d_temp = (float)temp16 / IQ8;

		    rxBuffer[i].valid = 0;
			break;
		}

		if(get_CVP && get_Vbat && get_Mtemp && get_Dtemp) return true;

	}

	return false;
}
////////////////////////////////////////////////////////////////////
// GYEMS functions
////////////////////////////////////////////////////////////////////
void INNFOS_Init(uint8_t id){
	innfos_data[0] = 0x2A; innfos_data[1] = 0x01; INNFOS_sendFrame(id, innfos_data, 2); // SCA enable
	sleep(1);
	innfos_data[0] = 0x07; innfos_data[1] = 0x06; INNFOS_sendFrame(id, innfos_data, 2); // Select usage mode [ position loop ]
}

void INNFOS_deInit(uint8_t id){
	innfos_data[0] = 0x2A; innfos_data[1] = 0x00; INNFOS_sendFrame(id, innfos_data, 2); // SCA disable
}

bool INNFOS_posCmd(INNFOS_REPLY* reply,
						  uint8_t id,
						  float position,
						  uint16_t timeout)
{

	float temp;
	int32_t temp32;

	// Send frame
	temp = position * IQ24;
	temp /= M_PI2;
	temp *= 36;
	temp32 = (int32_t)temp;

	innfos_data[0] = 0x0A;
	innfos_data[1] = (uint8_t)(temp32 >> 24);
	innfos_data[2] = (uint8_t)(temp32 >> 16);
	innfos_data[3] = (uint8_t)(temp32 >> 8);
	innfos_data[4] = (uint8_t)temp32;
	INNFOS_sendFrame(id, innfos_data, 5);

	// Get position / velocity / current value
	innfos_data[0] = 0x94;
	INNFOS_sendFrame(id, innfos_data, 1);

	// Get voltage value
	innfos_data[0] = 0x45;
	INNFOS_sendFrame(id, innfos_data, 1);

	// Get motor temperature value
	innfos_data[0] = 0x5F;
	INNFOS_sendFrame(id, innfos_data, 1);

	// Get driver temperature value
	innfos_data[0] = 0x60;
	INNFOS_sendFrame(id, innfos_data, 1);

	rx_timeout = timeout;
	return INNFOS_parseResponse(reply, id);

}


