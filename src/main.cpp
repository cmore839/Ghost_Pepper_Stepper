#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
<<<<<<< HEAD
#include <math.h>
=======
#include <math.h> // For pow() function in auto-tune
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
<<<<<<< HEAD
#include "stm32g4xx_hal_conf.h"
#include "InlineCurrentSenseSync.h"
#include "can.h"
=======

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#include "can.h"
#include "dfu.h"
#include "utils.h"
#include "InlineCurrentSenseSync.h"
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7

// Pin Definitions
#define ENC_COPI PA7
#define ENC_CIPO PA6
#define ENC_CS   PC4
#define ENC_SCK  PA5
#define MOT_EN   PB12
#define MOT_A1   PA0
#define MOT_A2   PA10
#define MOT_B1   PA9
#define MOT_B2   PA1
#define ISENSE_V PA3
#define ISENSE_U PB13
#define ISENSE_W PB0
<<<<<<< HEAD
=======
#define CAN_TX   PB9
#define CAN_RX   PB8
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
#define LED_FAULT  PB11

// Motor & Control Parameters
#define POLEPAIRS 7
#define RPHASE 0.6
#define MOTORKV 360
#define q_phase_inductance 0.0015
#define d_phase_inductance 0.0015

<<<<<<< HEAD
=======
// Application State
bool motor_enabled = false;
float target_value = 0;
uint8_t telemetry_mode = 0;
bool autotune_running = false;

// Telemetry Timing
unsigned long last_telemetry_time = 0;
unsigned int telemetry_period_ms = 10;

>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
// SimpleFOC Objects
BLDCDriver3PWM DR1 = BLDCDriver3PWM(MOT_A1, MOT_A2, MOT_B1, MOT_EN);
BLDCMotor M1 = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
SPISettings myMT6835SPISettings(12000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 E1 = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
InlineCurrentSenseSync CS1 = InlineCurrentSenseSync(0.005, 50, ISENSE_V, ISENSE_U, ISENSE_W);

// EEPROM Data Structure
typedef struct {
    uint16_t signature;
    Direction sensor_direction;
    float zero_electric_angle;
    uint8_t encoder_calibrated;
    uint8_t can_id;
} UserData;

UserData board_data;
const uint16_t EEPROM_MAGIC_WORD = 0xAF0C;

<<<<<<< HEAD
// Application State
bool motor_enabled = false;
float target_angle = 0.0f;
unsigned long last_telemetry_time = 0;

=======
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
// Function Prototypes
void loadSettings();
void saveSettings();
void runCalibrationSequence();
<<<<<<< HEAD
void send_param_response(uint8_t param_id, float value);
=======
void processCANCommands();
void sendAdvancedCANTelemetry(int current_loop_time);
void sendRegisterValue(uint8_t register_id, float value);
void runAutoTuneSequence();
float normalizeAngle(float angle);
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7

void setup() {
    pinMode(LED_FAULT, OUTPUT);
    digitalWrite(LED_FAULT, HIGH);
    Serial3.begin(115200);
    SimpleFOCDebug::enable(&Serial3);
<<<<<<< HEAD
    SIMPLEFOC_DEBUG("--- Core Firmware with Minimal PID CAN ---");

    loadSettings();
    
    CAN_Init(board_data.can_id);
    SIMPLEFOC_DEBUG("Motor CAN ID set to:", board_data.can_id);
=======

    loadSettings();

    if (board_data.can_id == 0 || board_data.can_id > 127) {
        CAN_Init(0);
        uint8_t foundID = CAN_FindUniqueID();
        if (foundID != 0) { board_data.can_id = foundID; saveSettings(); }
        else { while (true); }
    }
    CAN_Init(board_data.can_id);
    SIMPLEFOC_DEBUG("CAN configured with ID: 0x%02X", board_data.can_id);
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
    
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

<<<<<<< HEAD
void loop() {
    M1.loopFOC();

    if (motor_enabled) {
        M1.move(target_angle);
    }

    // --- CAN Command Polling ---
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (CAN_Poll(&rxHeader, rxData)) {
        if (rxHeader.Identifier == board_data.can_id) {
            SIMPLEFOC_DEBUG("CAN Command Received!");
            CommandType cmd = (CommandType)rxData[0];
            float payload_float;
            
            if (rxHeader.DataLength >= 5) { // Commands with a float payload
                memcpy(&payload_float, &rxData[1], sizeof(float));
            }
            
            switch(cmd) {
                case CMD_SET_ENABLE:
                    motor_enabled = (rxData[1] == 1);
                    if (motor_enabled) { M1.enable(); SIMPLEFOC_DEBUG("Motor ENABLED"); }
                    else { M1.disable(); SIMPLEFOC_DEBUG("Motor DISABLED"); }
                    break;
                case CMD_SET_TARGET:
                    target_angle = payload_float;
                    SIMPLEFOC_DEBUG("New Target:", target_angle);
                    break;
                case CMD_SET_VEL_P:
                    M1.PID_velocity.P = payload_float;
                    SIMPLEFOC_DEBUG("Set Vel P:", payload_float);
                    break;
                case CMD_SET_VEL_I:
                    M1.PID_velocity.I = payload_float;
                    SIMPLEFOC_DEBUG("Set Vel I:", payload_float);
                    break;
                case CMD_SET_ANGLE_P:
                    M1.P_angle.P = payload_float;
                    SIMPLEFOC_DEBUG("Set Angle P:", payload_float);
                    break;
                case CMD_REQUEST_PARAM:
                    uint8_t param_to_send = rxData[1];
                    if (param_to_send == CMD_SET_VEL_P) send_param_response(param_to_send, M1.PID_velocity.P);
                    if (param_to_send == CMD_SET_VEL_I) send_param_response(param_to_send, M1.PID_velocity.I);
                    if (param_to_send == CMD_SET_ANGLE_P) send_param_response(param_to_send, M1.P_angle.P);
                    break;
            }
        }
    }

    // --- CAN Telemetry Sending ---
    if (millis() - last_telemetry_time > 10) { // Send at ~100Hz
        last_telemetry_time = millis();
        
        float current_angle = M1.shaft_angle;
        uint8_t telemetry_data[4];
        memcpy(telemetry_data, &current_angle, sizeof(float));
        
        CAN_Send(CAN_ID_MOTOR_1_TELEMETRY, telemetry_data, FDCAN_DLC_BYTES_4);
    }
}

void send_param_response(uint8_t param_id, float value) {
    uint8_t response_data[5];
    response_data[0] = param_id;
    memcpy(&response_data[1], &value, sizeof(float));
    CAN_Send(CAN_ID_REGISTER_RESPONSE, response_data, FDCAN_DLC_BYTES_5);
=======
float normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

void loop() {
    unsigned long loop_start_us = micros();

    if (autotune_running) {
        runAutoTuneSequence();
        autotune_running = false; 
        
        // FIX: Flush any stale CAN commands that arrived during the blocking tune
        SIMPLEFOC_DEBUG("Flushing CAN RX buffer...");
        FDCAN_RxHeaderTypeDef dummyHeader;
        uint8_t dummyData[8];
        while(CAN_Poll(&dummyHeader, dummyData)) { } // Poll until buffer is empty
        SIMPLEFOC_DEBUG("CAN RX buffer flushed.");
        
        return; // Skip the rest of this loop iteration
    }

    processCANCommands(); 

    M1.loopFOC();
    if (motor_enabled) {
        M1.move(target_value);
    }
    
    if (telemetry_mode > 0 && (millis() - last_telemetry_time > telemetry_period_ms)) {
        last_telemetry_time = millis();
        int current_loop_time = micros() - loop_start_us;
        if (telemetry_mode == 2) {
            sendAdvancedCANTelemetry(current_loop_time);
        }
    }
}

void processCANCommands() {
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (CAN_Poll(&rxHeader, rxData)) {
        CanCommandType cmd_type = (CanCommandType)rxData[0];
        float payload_float = 0;
        uint8_t payload_uint8 = 0;

        if (rxHeader.DataLength == 5) {
            memcpy(&payload_float, &rxData[1], sizeof(float));
        } else if (rxHeader.DataLength >= 2) {
            payload_uint8 = rxData[1];
        }

        switch (cmd_type) {
            case CMD_START_AUTOTUNE: autotune_running = true; break;
            case CMD_SET_TARGET: target_value = payload_float; break;
            case CMD_SET_ENABLE:
                motor_enabled = (payload_uint8 == 1);
                if (motor_enabled) M1.enable(); else M1.disable();
                break;
            case CMD_SET_MOTION_CONTROL_TYPE: M1.controller = (MotionControlType)payload_uint8; break;
            case CMD_SET_TORQUE_TYPE: M1.torque_controller = (TorqueControlType)payload_uint8; break;
            case CMD_SET_VEL_P: M1.PID_velocity.P = payload_float; break;
            case CMD_SET_VEL_I: M1.PID_velocity.I = payload_float; break;
            case CMD_SET_VEL_D: M1.PID_velocity.D = payload_float; break;
            case CMD_SET_VEL_RAMP: M1.PID_velocity.output_ramp = payload_float; break;
            case CMD_SET_VEL_LPF_TF: M1.LPF_velocity.Tf = payload_float; break;
            case CMD_SET_ANGLE_P: M1.P_angle.P = payload_float; break;
            case CMD_SET_CURQ_P: M1.PID_current_q.P = payload_float; break;
            case CMD_SET_CURQ_I: M1.PID_current_q.I = payload_float; break;
            case CMD_SET_CURQ_D: M1.PID_current_q.D = payload_float; break;
            case CMD_SET_CURQ_LPF_TF: M1.LPF_current_q.Tf = payload_float; break;
            case CMD_SET_CURD_P: M1.PID_current_d.P = payload_float; break;
            case CMD_SET_CURD_I: M1.PID_current_d.I = payload_float; break;
            case CMD_SET_CURD_D: M1.PID_current_d.D = payload_float; break;
            case CMD_SET_CURD_LPF_TF: M1.LPF_current_d.Tf = payload_float; break;
            case CMD_SET_VOLTAGE_LIMIT: M1.voltage_limit = payload_float; break;
            case CMD_SET_CURRENT_LIMIT: M1.current_limit = payload_float; break;
            case CMD_SET_VELOCITY_LIMIT: M1.velocity_limit = payload_float; break;
            case CMD_ADV_TELEMETRY_CONTROL: telemetry_mode = payload_uint8 > 0 ? 2 : 0; break;
            case CMD_SET_TELEMETRY_RATE:
                telemetry_period_ms = payload_uint8;
                if (telemetry_period_ms < 1) telemetry_period_ms = 1;
                break;
            case CMD_REQUEST_REGISTER:
                {
                    uint8_t requested_reg = payload_uint8;
                    switch(requested_reg) {
                        case CMD_SET_VEL_P: sendRegisterValue(requested_reg, M1.PID_velocity.P); break;
                        case CMD_SET_VEL_I: sendRegisterValue(requested_reg, M1.PID_velocity.I); break;
                        case CMD_SET_VEL_D: sendRegisterValue(requested_reg, M1.PID_velocity.D); break;
                        case CMD_SET_VEL_RAMP: sendRegisterValue(requested_reg, M1.PID_velocity.output_ramp); break;
                        case CMD_SET_VEL_LPF_TF: sendRegisterValue(requested_reg, M1.LPF_velocity.Tf); break;
                        case CMD_SET_ANGLE_P: sendRegisterValue(requested_reg, M1.P_angle.P); break;
                        case CMD_SET_VOLTAGE_LIMIT: sendRegisterValue(requested_reg, M1.voltage_limit); break;
                        case CMD_SET_CURRENT_LIMIT: sendRegisterValue(requested_reg, M1.current_limit); break;
                        case CMD_SET_VELOCITY_LIMIT: sendRegisterValue(requested_reg, M1.velocity_limit); break;
                    }
                }
                break;
            case CMD_RECALIBRATE: runCalibrationSequence(); break;
            case CMD_JUMP_TO_DFU: jump_to_bootloader(); break;
        }
    }
}

void runAutoTuneSequence() {
    SIMPLEFOC_DEBUG("Starting Auto-Tune Frequency Sweep...");
    digitalWrite(LED_FAULT, HIGH);

    MotionControlType original_controller = M1.controller;
    TorqueControlType original_torque = M1.torque_controller;
    float original_voltage_limit = M1.voltage_limit;

    M1.controller = MotionControlType::torque;
    M1.torque_controller = TorqueControlType::voltage;
    M1.voltage_limit = 2.0;

    float start_freq = 5.0;
    float end_freq = 500.0;
    int steps = 50;
    float voltage_amplitude = 1.0;

    for (int i = 0; i < steps; i++) {
        float freq = start_freq * pow(end_freq / start_freq, (float)i / (steps - 1));
        float omega = freq * _2PI;
        
        unsigned long t_start = micros();
        float max_vel = 0;
        unsigned long sweep_duration_ms = 200;
        while(micros() - t_start < sweep_duration_ms * 1000) {
            float angle = normalizeAngle( (float)(micros() - t_start) / 1000000.0f * omega);
            M1.move(voltage_amplitude * cos(angle));
            M1.loopFOC();
            if (abs(M1.shaft_velocity) > max_vel) {
                max_vel = abs(M1.shaft_velocity);
            }
        }
        
        uint8_t response_data[8];
        float magnitude = max_vel / voltage_amplitude;
        memcpy(&response_data[0], &freq, sizeof(float));
        memcpy(&response_data[4], &magnitude, sizeof(float));
        CAN_Send(board_data.can_id + 104, response_data, FDCAN_DLC_BYTES_8);
        delay(20);
    }

    uint8_t done_data[1] = {0xFF};
    CAN_Send(board_data.can_id + 104, done_data, FDCAN_DLC_BYTES_1);

    M1.controller = original_controller;
    M1.torque_controller = original_torque;
    M1.voltage_limit = original_voltage_limit;
    digitalWrite(LED_FAULT, LOW);
    SIMPLEFOC_DEBUG("Auto-Tune complete.");
}

void sendRegisterValue(uint8_t register_id, float value) {
    uint8_t response_data[5];
    response_data[0] = register_id;
    memcpy(&response_data[1], &value, sizeof(float));
    CAN_Send(board_data.can_id + 200, response_data, FDCAN_DLC_BYTES_5);
}

void sendAdvancedCANTelemetry(int current_loop_time) {
    uint8_t frame1_data[8];
    memcpy(&frame1_data[0], &M1.shaft_angle, sizeof(float));
    memcpy(&frame1_data[4], &M1.shaft_velocity, sizeof(float));
    CAN_Send(board_data.can_id + 101, frame1_data, FDCAN_DLC_BYTES_8);

    uint8_t frame2_data[8];
    memcpy(&frame2_data[0], &M1.current.q, sizeof(float));
    memcpy(&frame2_data[4], &M1.current.d, sizeof(float));
    CAN_Send(board_data.can_id + 102, frame2_data, FDCAN_DLC_BYTES_8);

    uint8_t frame3_data[2];
    uint16_t loop_us = (uint16_t)current_loop_time;
    memcpy(&frame3_data[0], &loop_us, sizeof(uint16_t));
    CAN_Send(board_data.can_id + 103, frame3_data, FDCAN_DLC_BYTES_2);
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
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
<<<<<<< HEAD
        board_data.can_id = 1;
=======
        board_data.can_id = 0;
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
        saveSettings();
    }
}

void saveSettings() {
    EEPROM.put(0, board_data);
<<<<<<< HEAD
}
=======
}
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
