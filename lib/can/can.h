<<<<<<< HEAD
#ifndef MINIMAL_CAN_H
#define MINIMAL_CAN_H
=======
#ifndef AIOLI_CAN_H
#define AIOLI_CAN_H
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

<<<<<<< HEAD
// --- CAN Command and Telemetry IDs ---
#define CAN_ID_MOTOR_1_COMMAND   0x01
#define CAN_ID_MOTOR_1_TELEMETRY 0x101
#define CAN_ID_REGISTER_RESPONSE 0x201 // For responding to parameter requests

// Command IDs
enum CommandType : uint8_t {
    CMD_SET_ENABLE    = 0x01, // [uint8_t enabled]
    CMD_SET_TARGET    = 0x02, // [float target]
    CMD_SET_VEL_P     = 0x10, // [float value]
    CMD_SET_VEL_I     = 0x11, // [float value]
    CMD_SET_ANGLE_P   = 0x20, // [float value]
    CMD_REQUEST_PARAM = 0x81  // [uint8_t param_id_to_request]
};

void CAN_Init(uint8_t can_id);
void CAN_Send(uint16_t id, uint8_t* data, uint32_t dlc);
bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);

#endif // MINIMAL_CAN_H
=======
// CAN command IDs based on SimpleFOCRegisters.h
enum CanCommandType : uint8_t {
    // Motion Control
    CMD_SET_TARGET              = 0x01, // float
    CMD_SET_ENABLE              = 0x02, // uint8_t
    CMD_SET_MOTION_CONTROL_TYPE = 0x03, // uint8_t
    CMD_SET_TORQUE_TYPE         = 0x04, // uint8_t

    // Velocity PID & LPF
    CMD_SET_VEL_P               = 0x10, // float
    CMD_SET_VEL_I               = 0x11, // float
    CMD_SET_VEL_D               = 0x12, // float
    CMD_SET_VEL_RAMP            = 0x13, // float
    CMD_SET_VEL_LPF_TF          = 0x14, // float

    // Angle Controller
    CMD_SET_ANGLE_P             = 0x20, // float

    // Current (q-axis) PID & LPF
    CMD_SET_CURQ_P              = 0x30, // float
    CMD_SET_CURQ_I              = 0x31, // float
    CMD_SET_CURQ_D              = 0x32, // float
    CMD_SET_CURQ_LPF_TF         = 0x33, // float

    // Current (d-axis) PID & LPF
    CMD_SET_CURD_P              = 0x40, // float
    CMD_SET_CURD_I              = 0x41, // float
    CMD_SET_CURD_D              = 0x42, // float
    CMD_SET_CURD_LPF_TF         = 0x43, // float

    // Limits
    CMD_SET_VOLTAGE_LIMIT       = 0x50, // float
    CMD_SET_CURRENT_LIMIT       = 0x51, // float
    CMD_SET_VELOCITY_LIMIT      = 0x52, // float
    
    // Telemetry & System
    CMD_TELEMETRY_CONTROL       = 0x80, // uint8_t
    CMD_REQUEST_REGISTER        = 0x81, // uint8_t (payload is the register to read)
    CMD_REGISTER_RESPONSE       = 0x82, // Response from motor with register data
    CMD_ADV_TELEMETRY_CONTROL   = 0x83, // New command for advanced telemetry
    CMD_SET_TELEMETRY_RATE      = 0x84, // New command to set telemetry period in ms
    CMD_START_AUTOTUNE          = 0x85, // New command to start the auto-tuning sequence
    CMD_AUTOTUNE_RESPONSE       = 0x86, // Response with frequency sweep data

    CMD_JUMP_TO_DFU             = 0xF0,
    CMD_RECALIBRATE             = 0xF1
};

// Global CAN data buffer for transmitting
extern volatile uint8_t TxData[8];

void CAN_Init(uint8_t can_id);
void CAN_Send(uint16_t id, uint8_t* data, uint32_t dlc);
uint8_t CAN_FindUniqueID();
bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);

#endif // AIOLI_CAN_H
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
