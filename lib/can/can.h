#ifndef AIOLI_CAN_H
#define AIOLI_CAN_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

// CAN command IDs
enum CanCommandType : uint8_t {
    CMD_NOP                 = 0x00,
    CMD_SET_ANGLE           = 0x01,
    CMD_SET_ENABLED         = 0x02,
    CMD_JUMP_TO_DFU         = 0x10,
    CMD_RECALIBRATE         = 0x11,
    CMD_TELEMETRY_CONTROL   = 0x12
};

// Global CAN data buffer for transmitting
extern volatile uint8_t TxData[8];

void CAN_Init(uint8_t can_id);
void CAN_Send(uint16_t id, uint8_t* data, uint32_t dlc);
uint8_t CAN_FindUniqueID();

// FIX: Simplified function signature
bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);

#endif // AIOLI_CAN_H