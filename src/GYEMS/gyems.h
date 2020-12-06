#ifndef WATT_CAN_SRC_GYEMS_GYEMS_H_
#define WATT_CAN_SRC_GYEMS_GYEMS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _GYEMS_REPLY{
	int8_t temperature;
	int16_t iq;
	int16_t speed;
	int16_t encoder;
} GYEMS_REPLY;

extern bool GYEMS_posCmd1(GYEMS_REPLY *reply,
						  uint8_t id,
						  uint32_t angleControl,
						  uint16_t timeout);

extern bool GYEMS_posCmd2(GYEMS_REPLY *reply,
		                  uint8_t id,
		                  uint16_t maxSpeed,
						  uint32_t angleControl,
						  uint16_t timeout);

extern bool GYEMS_posCmd3(GYEMS_REPLY *reply,
		                  uint8_t id,
		                  uint8_t spinDirection,
						  uint16_t angleControl,
						  uint16_t timeout);

extern bool GYEMS_posCmd4(GYEMS_REPLY *reply,
		                  uint8_t id,
		                  uint8_t spinDirection,
						  uint16_t maxSpeed,
						  uint16_t angleControl,
						  uint16_t timeout);

extern void GYEMS_addRxBuffer(uint16_t cobID, uint8_t* data);
extern void GYEMS_timerLoop();

#ifdef __cplusplus
}
#endif

#endif /* WATT_CAN_SRC_GYEMS_GYEMS_H_ */
