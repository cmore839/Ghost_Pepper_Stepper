#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#include "can.h"
#include "dfu.h"
#include "utils.h"
#include "InlineCurrentSenseSync.h"

// Pin Definitions
#define ENC_COPI PA7
#define ENC_CIPO PA6
#define ENC_CS   PC4
#define ENC_SCK  PA5
#define CAL_EN   PA4

#define MOT_A1   PA0
#define MOT_A2   PA10
#define MOT_B1   PA9
#define MOT_B2   PA1
#define MOT_EN   PB12

#define ISENSE_V PA3
#define ISENSE_U PB13
#define ISENSE_W PB0

#define CAN_TX   PB9
#define CAN_RX   PB8

#define BUTTON_PIN PC15
#define LED_FAULT  PB11

// Motor & Control Parameters
#define POLEPAIRS 7
#define RPHASE 0.6
#define MOTORKV 360
#define q_phase_inductance 0.0015
#define d_phase_inductance 0.0015

// Application State
bool motor_enabled = false;
float target_angle = 0;
bool telemetry_enabled = false;

// Loop Timing & Debugging
int looptime = 0;
int loopcounter = 0;
const int loopiter = 20;

// SimpleFOC Objects
BLDCDriver3PWM DR1 = BLDCDriver3PWM(MOT_A1, MOT_A2, MOT_B1, MOT_EN);
BLDCMotor M1 = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
SPISettings myMT6835SPISettings(12000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 E1 = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
InlineCurrentSenseSync CS1 = InlineCurrentSenseSync(0.005, 50, ISENSE_V, ISENSE_U, ISENSE_W);

// EEPROM Data Structure
typedef struct
{
    uint16_t signature;
    Direction sensor_direction;
    float zero_electric_angle;
    uint8_t encoder_calibrated;
    uint8_t can_id;
} UserData;

UserData board_data;
const uint16_t EEPROM_MAGIC_WORD = 0xAF0C;

// Function Prototypes
void loadSettings();
void saveSettings();
void runCalibrationSequence();
void processCANCommands();
void sendCANTelemetry();

void setup() {
    pinMode(LED_FAULT, OUTPUT);
    digitalWrite(LED_FAULT, HIGH);
    Serial3.begin(115200);
    SimpleFOCDebug::enable(&Serial3);

    loadSettings();

    if (board_data.can_id == 0 || board_data.can_id > 127) {
        CAN_Init(0);
        uint8_t foundID = CAN_FindUniqueID();
        if (foundID != 0) { board_data.can_id = foundID; saveSettings(); }
        else { while (true); }
    }
    CAN_Init(board_data.can_id);
    SIMPLEFOC_DEBUG("CAN configured with ID: 0x%02X", board_data.can_id);
    
    // Original Tuning Parameters
    float current_bandwidth = 100.0; 
    float sample_frequency = (1000000 / 50);
    float Tf_ratio = 5.0; 
    float vel_bandwidth_ratio = 0.1;
    float vel_PI_ratio = 10.0; 
    float pos_bandwidth_ratio = 0.1;
    float pos_PI_ratio = 10.0;

    M1.velocity_limit = 99999;
    M1.voltage_limit = 1.0;
    M1.current_limit = 2.0;
    DR1.pwm_frequency = 20000;
    DR1.voltage_power_supply = 8.0;
    M1.voltage_sensor_align = 1.0;

    M1.PID_current_q.P = q_phase_inductance*current_bandwidth*_2PI;
    M1.PID_current_q.I= M1.PID_current_q.P*RPHASE/q_phase_inductance;
    M1.PID_current_d.P= d_phase_inductance*current_bandwidth*_2PI;
    M1.PID_current_d.I = M1.PID_current_d.P*RPHASE/d_phase_inductance;
    M1.LPF_current_q.Tf = 1/(Tf_ratio*current_bandwidth); 
    M1.LPF_current_d.Tf = 1/(Tf_ratio*current_bandwidth);
    M1.motion_downsample = 0;

    M1.PID_velocity.P = M1.PID_current_q.P * vel_bandwidth_ratio;
    M1.PID_velocity.I = M1.PID_velocity.P*vel_PI_ratio;
    M1.LPF_velocity.Tf = (1/(Tf_ratio*M1.PID_velocity.P*sample_frequency)); 
   
    M1.P_angle.P = M1.PID_velocity.P*pos_bandwidth_ratio*sample_frequency;
    M1.P_angle.I = (M1.P_angle.P*pos_PI_ratio)/sample_frequency;
    
    E1.init();
    M1.linkSensor(&E1);
    DR1.init();
    M1.linkDriver(&DR1);
    CS1.linkDriver(&DR1);
    CS1.init();
    M1.linkCurrentSense(&CS1);
    
    M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    M1.controller = MotionControlType::angle;
    M1.torque_controller = TorqueControlType::foc_current;
    
    M1.init();
    
    if (!board_data.encoder_calibrated) {
        runCalibrationSequence();
    } else {
        M1.sensor_direction = board_data.sensor_direction;
        M1.zero_electric_angle = board_data.zero_electric_angle;
    }

    M1.initFOC();
    M1.disable(); 
    SIMPLEFOC_DEBUG("Setup complete. Motor ready.");
    digitalWrite(LED_FAULT, LOW);
}

void loop() {
    unsigned long timestamp_start = micros();

    processCANCommands(); 

    M1.loopFOC();
    if (motor_enabled) {
        M1.move(target_angle);
    }
    
    if (loopcounter >= loopiter){
        looptime = micros() - timestamp_start;
        if (telemetry_enabled) {
            sendCANTelemetry();
        }
        loopcounter = 0;
    }
    loopcounter++;
}

void processCANCommands() {
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Poll for a new message. If found, header and data are filled.
    if (CAN_Poll(&rxHeader, rxData)) {
        CanCommandType cmd_type = (CanCommandType)rxData[0];

        switch (cmd_type) {
            case CMD_SET_ANGLE:
                memcpy(&target_angle, &rxData[1], sizeof(float));
                break;
            case CMD_SET_ENABLED:
                motor_enabled = (rxData[1] == 1);
                if (motor_enabled) { M1.enable(); } 
                else { M1.disable(); }
                break;
            case CMD_TELEMETRY_CONTROL:
                telemetry_enabled = (rxData[1] == 1);
                SIMPLEFOC_DEBUG("Telemetry control received:", telemetry_enabled);
                break;
            case CMD_RECALIBRATE:
                runCalibrationSequence();
                break;
            case CMD_JUMP_TO_DFU:
                jump_to_bootloader(); 
                break;
        }
    }
}

void sendCANTelemetry() {
    uint8_t telemetry_data[6];
    float current_angle = M1.shaft_angle;
    uint16_t current_loop_time = (uint16_t)looptime;

    memcpy(&telemetry_data[0], &current_angle, sizeof(float));
    memcpy(&telemetry_data[4], &current_loop_time, sizeof(uint16_t));
    
    CAN_Send(board_data.can_id + 100, telemetry_data, FDCAN_DLC_BYTES_6);
}

void runCalibrationSequence() {
    SIMPLEFOC_DEBUG("Starting calibration...");
    digitalWrite(LED_FAULT, HIGH);
    M1.controller = MotionControlType::angle_openloop;
    M1.voltage_sensor_align = 2.0;
    M1.initFOC();
    board_data.sensor_direction = M1.sensor_direction;
    board_data.zero_electric_angle = M1.zero_electric_angle;
    board_data.encoder_calibrated = 1;
    saveSettings();
    M1.controller = MotionControlType::angle;
    SIMPLEFOC_DEBUG("Calibration complete.");
    digitalWrite(LED_FAULT, LOW);
}

void loadSettings() {
    EEPROM.get(0, board_data);
    if (board_data.signature != EEPROM_MAGIC_WORD) {
        board_data.signature = EEPROM_MAGIC_WORD;
        board_data.sensor_direction = UNKNOWN;
        board_data.zero_electric_angle = 0.0f;
        board_data.encoder_calibrated = 0;
        board_data.can_id = 0;
        saveSettings();
    }
}

void saveSettings() {
    EEPROM.put(0, board_data);
}