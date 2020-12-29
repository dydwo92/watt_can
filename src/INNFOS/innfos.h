#ifndef WATT_CAN_SRC_INNFOS_H_
#define WATT_CAN_SRC_INNFOS_H_
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define Velocity_Max	6000.0f
#define BlkEngy_Scal	75.225f
#define Profile_Scal	960.0f
#define IQ8				256.0f			//2^8
#define IQ10			1024.0f			//2^10
#define IQ24			16777216.0f		//2^24
#define IQ30			1073741824.0f	//2^30

#define M_PI2			6.283185307f

#define BUFLEN            40
#define RX_TIMEOUT        1000

#define ID_BASE	0xC0
#define NUM_MOTOR	6

typedef struct _INNFOS_REPLY{
	float m_temp; // C
	float d_temp; // C
	float Voltage; // V
	float Current; // A
	float Speed; // rad/s
	float Position; // rad
} INNFOS_REPLY;

extern void INNFOS_Init(uint8_t id);
extern void INNFOS_deInit(uint8_t id);
extern bool INNFOS_posCmd(INNFOS_REPLY* reply,
						  uint8_t id,
						  float position,
						  uint16_t timeout);

extern void INNFOS_addRxBuffer(uint8_t id, uint8_t* data);
extern void INNFOS_timerLoop();

#ifdef __cplusplus
}
#endif


#endif
