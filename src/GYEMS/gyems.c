#include "gyems.h"

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

#define BUFF_TIMEOUT	100

struct GYEMS_BUFF{
	struct GYEMS_BUFF * next;
	struct GYEMS_BUFF * prev;
	uint8_t id;
	uint8_t data[8];
	uint16_t timeout;
};

static uint8_t gyems_data[8];

volatile static uint16_t rx_timeout;

static uint8_t lock_buff = 0;
static struct GYEMS_BUFF* buff_head = NULL;
static struct GYEMS_BUFF* buff_tail = NULL;

////////////////////////////////////////////////////////////////////
// CAN Send/Get function
////////////////////////////////////////////////////////////////////
void GYEMS_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len){

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

void GYEMS_addRxBuffer(uint16_t cobID, uint8_t* data){

	struct GYEMS_BUFF* buffer;
	uint8_t id;

	if((cobID & 0x140) != 0x140) return;
	id = cobID & 0x3F;

	while(lock_buff);
	lock_buff = 1;

	// create new buffer
	buffer = malloc(sizeof(struct GYEMS_BUFF));
	if(!buffer){
		lock_buff = 0;
		return;
	}

	buffer->id = id;
	buffer->timeout = BUFF_TIMEOUT;
	memcpy(buffer->data, data, 8);
    buffer->next = NULL;
    buffer->prev = NULL;

    if(buff_head == NULL || buff_tail == NULL){
    	buff_head = buffer;
    	buff_tail = buffer;
    }else{
    	buffer->prev = buff_tail;
    	buff_tail->next = buffer;
    	buff_tail = buffer;
    }

	lock_buff = 0;
}

void GYEMS_timerLoop(){

	struct GYEMS_BUFF *buff, *buff_tmp;

	while(lock_buff);
	lock_buff = 1;

	if(rx_timeout != 0) rx_timeout--;

	buff = buff_head;
	while(buff){
		buff_tmp = buff->next;
		if(--buff->timeout == 0){ // expire buffer
			if(buff->prev == NULL){ // Head
				buff_head = buff->next;
				if(buff->next) buff->next->prev = NULL;
			}else if(buff->next == NULL){ // Tail
				buff_tail = buff->prev;
				if(buff->prev) buff->prev->next = NULL;
			}else{
				buff->prev->next = buff->next;
				buff->next->prev = buff->prev;
			}
			free(buff);
		}

		buff = buff_tmp;

	}

	lock_buff = 0;
}

bool GYEMS_parseResponse(GYEMS_REPLY *reply, uint8_t id){

	struct GYEMS_BUFF *buff = buff_head;

	while(rx_timeout){

		if(buff == NULL){
			buff = buff_head;
			continue;
		}

		if(buff->id != id){
			buff = buff->next;
			continue;
		}

		reply->temperature = buff->data[1];
		reply->iq = (uint16_t)buff->data[3] << 8 | (uint16_t)buff->data[2];
		reply->speed = (uint16_t)buff->data[5] << 8 | (uint16_t)buff->data[4];
		reply->encoder = (uint16_t)buff->data[7] << 8 | (uint16_t)buff->data[6];

		// Remove frame
		while(lock_buff);
		lock_buff = 1;

		if(buff->prev == NULL){ // Head
			buff_head = buff->next;
			if(buff->next) buff->next->prev = NULL;
		}else if(buff->next == NULL){ // Tail
			buff_tail = buff->prev;
			if(buff->prev) buff->prev->next = NULL;
		}else{
			buff->prev->next = buff->next;
			buff->next->prev = buff->prev;
		}

		free(buff);
		lock_buff = 0;
		return true;

	}

	return false;
}
////////////////////////////////////////////////////////////////////
// GYEMS functions
////////////////////////////////////////////////////////////////////
bool GYEMS_posCmd1(GYEMS_REPLY *reply,
		           uint8_t id,
		  	  	   uint32_t angleControl,
				   uint16_t timeout)
{

	uint16_t cob_ID = 0x140 | (uint16_t)id;

	// Send frame
	gyems_data[0] = 0xA3;
	gyems_data[1] = 0x00;
	gyems_data[2] = 0x00;
	gyems_data[3] = 0x00;
	gyems_data[4] = (uint8_t)angleControl;
	gyems_data[5] = (uint8_t)(angleControl >> 8);
	gyems_data[6] = (uint8_t)(angleControl >> 16);
	gyems_data[7] = (uint8_t)(angleControl >> 24);
	GYEMS_sendFrame(cob_ID, gyems_data, 8);

	// Get frame
	rx_timeout = timeout;
	return GYEMS_parseResponse(reply, id);

}

bool GYEMS_posCmd2(GYEMS_REPLY *reply,
		           uint8_t id,
        	       uint16_t maxSpeed,
		           uint32_t angleControl,
		           uint16_t timeout)
{

	uint16_t cob_ID = 0x140 | (uint16_t)id;

	gyems_data[0] = 0xA4;
	gyems_data[1] = 0x00;
	gyems_data[2] = (uint8_t)maxSpeed;
	gyems_data[3] = (uint8_t)(maxSpeed >> 8);
	gyems_data[4] = (uint8_t)angleControl;
	gyems_data[5] = (uint8_t)(angleControl >> 8);
	gyems_data[6] = (uint8_t)(angleControl >> 16);
	gyems_data[7] = (uint8_t)(angleControl >> 24);
	GYEMS_sendFrame(cob_ID, gyems_data, 8);

	// Get frame
	rx_timeout = timeout;
	return GYEMS_parseResponse(reply, id);

}

bool GYEMS_posCmd3(GYEMS_REPLY *reply,
		           uint8_t id,
                   uint8_t spinDirection,
		           uint16_t angleControl,
		           uint16_t timeout)
{

	uint16_t cob_ID = 0x140 | (uint16_t)id;

	gyems_data[0] = 0xA5;
	gyems_data[1] = spinDirection & 0x01;
	gyems_data[2] = 0x00;
	gyems_data[3] = 0x00;
	gyems_data[4] = (uint8_t)angleControl;
	gyems_data[5] = (uint8_t)(angleControl >> 8);
	gyems_data[6] = 0x00;
	gyems_data[7] = 0x00;
	GYEMS_sendFrame(cob_ID, gyems_data, 8);

	// Get frame
	rx_timeout = timeout;
	return GYEMS_parseResponse(reply, id);

}

bool GYEMS_posCmd4(GYEMS_REPLY *reply,
		           uint8_t id,
                   uint8_t spinDirection,
		           uint16_t maxSpeed,
		           uint16_t angleControl,
		           uint16_t timeout)
{

	uint16_t cob_ID = 0x140 | (uint16_t)id;

	gyems_data[0] = 0xA6;
	gyems_data[1] = spinDirection & 0x01;
	gyems_data[2] = (uint8_t)maxSpeed;
	gyems_data[3] = (uint8_t)(maxSpeed >> 8);
	gyems_data[4] = (uint8_t)angleControl;
	gyems_data[5] = (uint8_t)(angleControl >> 8);
	gyems_data[6] = 0x00;
	gyems_data[7] = 0x00;
	GYEMS_sendFrame(cob_ID, gyems_data, 8);

	// Get frame
	rx_timeout = timeout;
	return GYEMS_parseResponse(reply, id);

}



