#include "CANOpen.h"

#include <string.h>

typedef struct CO_BUFFER{
  uint16_t cobID;
  uint8_t data[8];
  uint8_t valid;
  uint16_t timeout;
}CO_BUFFER;
static CO_BUFFER rxBuffer[CO_BUFLEN];

static uint16_t cobID;
static uint8_t txData[8];
static volatile uint16_t tx_timeout;

CO_Status CANOpen_sendSync(){
  // Create sync frame
  cobID = 0x80;
  
  // Send frame
  CANOpen_sendFrame(cobID, txData, 0);
  
  return CO_OK;
}

CO_Status CANOpen_writeOD(uint8_t nodeId,
                   uint16_t Index,
                   uint8_t subIndex,
                   uint8_t* data,
                   uint8_t len,
                   uint16_t timeout)
{
  uint16_t i;
  
  if(len > 4) len = 4; // Only support expedited transfer
  if(len == 0) return CO_OK;
  
  // Create rxSDO frame
  cobID = (uint16_t)(nodeId & 0x7F);
  cobID |= 0x0600;
  txData[0] = 0x23 | ((4 - len) << 2);
  txData[1] = (uint8_t)Index;
  txData[2] = (uint8_t)(Index >> 8);
  txData[3] = subIndex;
  memset(txData + 4, 0, 4);
  memcpy(txData + 4, data, len);
  
  // Send frame
  CANOpen_sendFrame(cobID, txData, 8);
  
  if(timeout == 0) return CO_OK;
  
  tx_timeout = timeout;
  while(tx_timeout != 0){
    // Find Valid response
    for(i = 0; i < CO_BUFLEN; i++){
      if(rxBuffer[i].valid == 0) continue;
      if(rxBuffer[i].cobID != (nodeId | 0x0580)) continue;
      if(rxBuffer[i].data[0] != 0x60) continue;
      if(rxBuffer[i].data[1] != txData[1]) continue;
      if(rxBuffer[i].data[2] != txData[2]) continue;
      if(rxBuffer[i].data[3] != txData[3]) continue;
      
      rxBuffer[i].valid = 0;
      return CO_OK;
    }
  }
  
  return CO_TIMEOUT;
}

CO_Status CANOpen_readOD(uint8_t nodeId,
                     uint16_t Index,
                     uint8_t subIndex,
                     uint8_t* data,
                     uint8_t* len,
                     uint16_t timeout)
{
  
  uint16_t i;
  
  // Create rxSDO frame
  cobID = (uint16_t)(nodeId & 0x7F);
  cobID |= 0x0600;
  txData[0] = 0x40;
  txData[1] = (uint8_t)Index;
  txData[2] = (uint8_t)(Index >> 8);
  txData[3] = subIndex;
  memset(txData + 4, 0, 4);

  // Send frame
  CANOpen_sendFrame(cobID, txData, 8);
  
  tx_timeout = timeout;
  while(tx_timeout != 0){
    // Find Valid response
    for(i = 0; i < CO_BUFLEN; i++){
      if(rxBuffer[i].valid == 0) continue;
      if(rxBuffer[i].cobID != (nodeId | 0x0580)) continue;
      if((rxBuffer[i].data[0] & 0x43) != 0x43) continue;
      if(rxBuffer[i].data[1] != txData[1]) continue;
      if(rxBuffer[i].data[2] != txData[2]) continue;
      if(rxBuffer[i].data[3] != txData[3]) continue;
      
      *len = 4 - ((rxBuffer[i].data[0] & 0x0C) >> 2);
      memcpy(data, rxBuffer[i].data + 4, *len);
      
      rxBuffer[i].valid = 0;
      return CO_OK;
    }
  }
  
  return CO_TIMEOUT;
}

CO_Status CANOpen_sendPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct){
  
  if(channel == 0 || channel > 4) return CO_ERROR;
  
  if(pdo_struct->mappinglen == 0) return CO_OK;
  
  // Create rxPDO frame
  cobID = (uint16_t)nodeId;
  switch(channel){
  case 1 : cobID |= 0x200; break;
  case 2 : cobID |= 0x300; break;
  case 3 : cobID |= 0x400; break;
  case 4 : cobID |= 0x500; break;
  }
  
  uint64_t tmp64 = 0;
  uint8_t i;
  uint8_t bytelen;
  
  uint8_t total_bit = 0;
  uint8_t total_byte;
  
  i = pdo_struct->mappinglen;
  while(1){
    i --;
    total_bit += pdo_struct->bitlen[i];
    bytelen = (pdo_struct->bitlen[i] - 1) / 8 + 1;
    tmp64 = tmp64 << pdo_struct->bitlen[i];
    switch(bytelen){
    case 1 : tmp64 |= *(uint8_t*)pdo_struct->data[i]; break;
    case 2 : tmp64 |= *(uint16_t*)pdo_struct->data[i]; break;
    case 4 : tmp64 |= *(uint32_t*)pdo_struct->data[i]; break;
    }
    
    if(i == 0) break;
  }
  
  for(i = 0; i < 8; i++){
    txData[i] = (uint8_t)tmp64;
    tmp64 = tmp64 >> 8;
  }

  // Send frame
  total_byte = (total_bit - 1) /8 + 1;
  CANOpen_sendFrame(cobID, txData, total_byte);
  
  return CO_OK;
}

uint16_t timeout_cnt;
CO_Status CANOpen_readPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct, uint16_t timeout){
  
  if(channel == 0 || channel > 4) return CO_ERROR;
  
  uint16_t cobID_target = nodeId;
  uint64_t tmp64 = 0;
  uint8_t i, j;
  uint8_t bytelen;
  
  switch(channel){
  case 1 : cobID_target |= 0x180; break;
  case 2 : cobID_target |= 0x280; break;
  case 3 : cobID_target |= 0x380; break;
  case 4 : cobID_target |= 0x480; break;
  }
  
  tx_timeout = timeout;
  while(tx_timeout != 0){
    // Find Valid frame
    for(i = 0; i < CO_BUFLEN; i++){
      if(rxBuffer[i].valid == 0) continue;
      if(rxBuffer[i].cobID != cobID_target) continue;
      
      memcpy(&tmp64, rxBuffer[i].data, 8);
      for(j = 0; j < pdo_struct->mappinglen; j++){
        bytelen = (pdo_struct->bitlen[j] - 1) / 8 + 1;
        memcpy(pdo_struct->data[j], &tmp64, bytelen);
        tmp64 = tmp64 >> pdo_struct->bitlen[j];
      }
      rxBuffer[i].valid = 0;
      return CO_OK;
    }
  }
  timeout_cnt++;
  return CO_TIMEOUT;
}

CO_Status CANOpen_writeOD_float(uint8_t nodeId, uint16_t Index, uint8_t subIndex, float data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 4, timeout);
}

CO_Status CANOpen_writeOD_uint32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint32_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 4, timeout);
}

CO_Status CANOpen_writeOD_int32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int32_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 4, timeout);
}

CO_Status CANOpen_writeOD_uint16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint16_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 2, timeout);
}

CO_Status CANOpen_writeOD_int16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int16_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 2, timeout);
}

CO_Status CANOpen_writeOD_uint8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, &data, 1, timeout);
}

CO_Status CANOpen_writeOD_int8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int8_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 1, timeout);
}

void CANOpen_mappingPDO_init(CO_PDOStruct* pdo_struct){
  pdo_struct->mappinglen = 0;
}

void CANOpen_mappingPDO(CO_PDOStruct* pdo_struct, void* data, uint8_t bitlen){
  pdo_struct->data[pdo_struct->mappinglen] = data;
  pdo_struct->bitlen[pdo_struct->mappinglen] = bitlen;
  pdo_struct->mappinglen ++;
}

void CANOpen_mappingPDO_float(CO_PDOStruct* pdo_struct, float* data){
  CANOpen_mappingPDO(pdo_struct, data, 32);
}

void CANOpen_mappingPDO_uint32(CO_PDOStruct* pdo_struct, uint32_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 32);
}

void CANOpen_mappingPDO_int32(CO_PDOStruct* pdo_struct, int32_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 32);
}

void CANOpen_mappingPDO_uint16(CO_PDOStruct* pdo_struct, uint16_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 16);
}

void CANOpen_mappingPDO_int16(CO_PDOStruct* pdo_struct, int16_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 16);
}

void CANOpen_mappingPDO_uint8(CO_PDOStruct* pdo_struct, uint8_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 8);
}

void CANOpen_mappingPDO_int8(CO_PDOStruct* pdo_struct, int8_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 8);
}


void CANOpen_addRxBuffer(uint16_t cobID, uint8_t* data){

  uint16_t i;
  for(i = 0; i < CO_BUFLEN; i++){
    if(rxBuffer[i].valid == 0){
      rxBuffer[i].timeout = CO_RX_TIMEOUT;
      rxBuffer[i].cobID = cobID;
      memcpy(rxBuffer[i].data, data, 8);
      rxBuffer[i].valid = 1;
      return;
    }
  }
  
}

void CANOpen_timerLoop(){ // Should be call every 1ms
  
  uint16_t i;
  if(tx_timeout != 0) tx_timeout --;
  for(i = 0; i < CO_BUFLEN; i++){
    if(rxBuffer[i].valid == 1){
      if(rxBuffer[i].timeout != 0) rxBuffer[i].timeout --;
      if(rxBuffer[i].timeout == 0) rxBuffer[i].valid = 0;
    }
  }
  
}
