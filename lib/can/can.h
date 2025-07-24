#ifndef MINIMAL_CAN_H
#define MINIMAL_CAN_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

// --- CAN IDs ---
#define CAN_ID_MOTOR_1_COMMAND   0x01
#define CAN_ID_MOTOR_1_TELEMETRY 0x101
#define CAN_ID_REGISTER_RESPONSE 0x201
#define CAN_ID_REQUEST_PARAM     0x81

// --- SimpleFOC Standard Register Map ---
typedef enum : uint8_t {
    REG_TARGET          = 0x01,
    REG_ENABLE          = 0x04,
    REG_CONTROL_MODE    = 0x05,
    REG_VEL_PID_P       = 0x30,
    REG_VEL_PID_I       = 0x31,
    REG_ANG_PID_P       = 0x36,
    REG_ANG_PID_I       = 0x37,
    REG_CURQ_PID_P      = 0x40,
    REG_CURQ_PID_I      = 0x41,
    REG_CURD_PID_P      = 0x46,
    REG_CURD_PID_I      = 0x47,
    REG_TELEMETRY_PERIOD= 0x82,
    REG_TELEMETRY_CONFIG= 0x83, // New register for the telemetry bitmask
} SimpleFOCRegister;

// --- Telemetry Control Bitmask ---
#define TELEM_ENABLE_ANGLE     (1 << 0) // Bit 0 = 1
#define TELEM_ENABLE_VELOCITY  (1 << 1) // Bit 1 = 2
#define TELEM_ENABLE_CURRENT_Q (1 << 2) // Bit 2 = 4
#define TELEM_ENABLE_LOOP_TIME (1 << 3) // Bit 3 = 8

// Telemetry IDs for multiplexing
enum TelemetryID : uint8_t {
    TELEM_ANGLE     = 0x01,
    TELEM_VELOCITY  = 0x02,
    TELEM_CURRENT_Q = 0x03,
    TELEM_LOOP_TIME = 0x04,
};

// Function Prototypes
bool CAN_Send(uint16_t id, uint8_t* data, uint32_t dlc);
bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);
void CAN_Init(uint8_t can_id);

#endif // MINIMAL_CAN_H