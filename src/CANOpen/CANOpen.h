#ifndef __CANOPENSLIM_H__
#define __CANOPENSLIM_H__

#include "CANOpen_hw_appl.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum CO_Status{
  CO_OK,
  CO_TIMEOUT,
  CO_ERROR,
}CO_Status;

typedef enum CO_NMT{
  CO_RESET,
  CO_COMMRESET,
  CO_STOPPED,
  CO_PREOP,
  CO_OP
}CO_NMT;

#define CO_PDOSTR_LEN        10
typedef struct CO_PDOStruct{
  void* data[CO_PDOSTR_LEN];
  uint8_t bitlen[CO_PDOSTR_LEN];
  uint8_t mappinglen;
}CO_PDOStruct;

/*****************************************************************************/
// CANOpen Interface Functions
/*****************************************************************************/
extern CO_Status CANOpen_sendSync();

extern CO_Status CANOpen_NMT(CO_NMT state);

extern CO_Status CANOpen_writeOD_float(uint8_t nodeId, uint16_t Index, uint8_t subIndex, float data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_uint32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint32_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_int32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int32_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_uint16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint16_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_int16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int16_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_uint8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_int8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int8_t data, uint16_t timeout);

extern CO_Status CANOpen_readOD(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t* data, uint8_t* len, uint16_t timeout);

extern void CANOpen_mappingPDO_init(CO_PDOStruct* pdo_struct);
extern void CANOpen_mappingPDO_float(CO_PDOStruct* pdo_struct, float* data);
extern void CANOpen_mappingPDO_uint32(CO_PDOStruct* pdo_struct, uint32_t* data);
extern void CANOpen_mappingPDO_int32(CO_PDOStruct* pdo_struct, int32_t* data);
extern void CANOpen_mappingPDO_uint16(CO_PDOStruct* pdo_struct, uint16_t* data);
extern void CANOpen_mappingPDO_int16(CO_PDOStruct* pdo_struct, int16_t* data);
extern void CANOpen_mappingPDO_uint8(CO_PDOStruct* pdo_struct, uint8_t* data);
extern void CANOpen_mappingPDO_int8(CO_PDOStruct* pdo_struct, int8_t* data);

extern CO_Status CANOpen_sendPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct);
extern CO_Status CANOpen_readPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct, uint16_t timeout);

/*****************************************************************************/
// User Install Functions
/*****************************************************************************/
extern void CANOpen_addRxBuffer(uint16_t cobID, uint8_t* data);
extern void CANOpen_timerLoop();

#ifdef __cplusplus
}
#endif

#endif
