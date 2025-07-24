#ifndef MINIMAL_CAN_H
#define MINIMAL_CAN_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

// --- CAN Command and Telemetry IDs ---
#define CAN_ID_MOTOR_1_COMMAND   0x01
#define CAN_ID_MOTOR_1_TELEMETRY 0x101
#define CAN_ID_REGISTER_RESPONSE 0x201

// Command IDs (from Python GUI)
enum CommandType : uint8_t {
    CMD_SET_ENABLE    = 0x01, // [uint8_t enabled]
    CMD_SET_TARGET    = 0x02, // [float target]
    CMD_SET_VEL_P     = 0x10, // [float value]
    CMD_SET_VEL_I     = 0x11, // [float value]
    CMD_SET_ANGLE_P   = 0x20, // [float value]
    CMD_REQUEST_PARAM = 0x81  // [uint8_t param_id_to_request]
};

// **NEW**: Telemetry IDs for multiplexing
enum TelemetryID : uint8_t {
    TELEM_ANGLE = 0x01,
    TELEM_VELOCITY = 0x02,
    TELEM_CURRENT_Q = 0x03,
};

void CAN_Init(uint8_t can_id);
// **MODIFIED**: Added a return type to check for send success
bool CAN_Send(uint16_t id, uint8_t* data, uint32_t dlc);
bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);

#endif // MINIMAL_CAN_H