// can.h
#ifndef MINIMAL_CAN_H
#define MINIMAL_CAN_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

// Correctly include the official SimpleFOC register definitions
#include <comms/SimpleFOCRegisters.h> 

// --- CAN IDs ---
#define CAN_ID_COMMAND_BASE         0x000
#define CAN_ID_MOTION_COMMAND_BASE  0x100
#define CAN_ID_TELEMETRY_BASE       0x200
#define CAN_ID_RESPONSE_BASE        0x300
#define CAN_ID_STATUS_FEEDBACK_BASE 0x400
#define CAN_ID_SCAN_BROADCAST       0x700
#define CAN_ID_SYNC                 0x080

// --- Custom Commands (in private range) ---
#define REG_CUSTOM_SAVE_TO_EEPROM        0xE1
#define REG_CUSTOM_FLIP_SENSOR_DIR       0xE2
#define REG_CUSTOM_TELEMETRY_PERIOD      0xE3
#define REG_CUSTOM_CHARACTERIZE_MOTOR    0xE4
#define REG_CUSTOM_SET_ID_AND_RESTART    0xE5 
#define REG_CUSTOM_MOTION_COMMAND        0xE6

// --- Function Prototypes ---
// *** THE FIX: Changed the third argument from uint32_t to uint8_t to match the .cpp file ***
bool CAN_Send(uint16_t id, uint8_t* data, uint8_t dlc_bytes);
bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);
void CAN_Init(uint8_t can_id);

#endif