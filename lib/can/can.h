// can.h
#ifndef MINIMAL_CAN_H
#define MINIMAL_CAN_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

// Instead of defining the registers here, we include the official header
#include <comms/SimpleFOCRegisters.h> 

// --- CAN IDs ---
#define CAN_ID_COMMAND_BASE         0x000
#define CAN_ID_TELEMETRY_BASE       0x200
#define CAN_ID_RESPONSE_BASE        0x300
#define CAN_ID_SCAN_BROADCAST       0x700

// --- Custom Commands (in private range) ---
#define REG_CUSTOM_SAVE_TO_EEPROM    0xE1
#define REG_CUSTOM_FLIP_SENSOR_DIR   0xE2
#define REG_CUSTOM_TELEMETRY_PERIOD  0xE3

// Function Prototypes
bool CAN_Send(uint16_t id, uint8_t* data, uint32_t dlc);
bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);
void CAN_Init(uint8_t can_id);

#endif