#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <math.h>

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "stm32g4xx_hal_conf.h"
#include "InlineCurrentSenseSync.h"
#include "can.h"

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
#define LED_FAULT  PB11

// Motor & Control Parameters
#define POLEPAIRS 7
#define RPHASE 0.6
#define MOTORKV 360
#define q_phase_inductance 0.0015
#define d_phase_inductance 0.0015

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

// Application State
bool motor_enabled = false;
float target_value = 0.0f;
unsigned long last_telemetry_time = 0;
int telemetry_state = 0;
unsigned long telemetry_period = 10;
unsigned long loop_duration_us = 0;
uint8_t telemetry_mask = TELEM_ENABLE_ANGLE | TELEM_ENABLE_VELOCITY | TELEM_ENABLE_CURRENT_Q | TELEM_ENABLE_LOOP_TIME;

// Function Prototypes
void loadSettings();
void saveSettings();
void runCalibrationSequence();
void send_param_response(uint8_t param_id, float value);

void setup() {
    pinMode(LED_FAULT, OUTPUT);
    digitalWrite(LED_FAULT, HIGH);
    
    Serial3.begin(115200);
    SimpleFOCDebug::enable(&Serial3);
    SIMPLEFOC_DEBUG("--- Core Firmware: High-Frequency CAN (Optimized) ---");

    loadSettings();
    CAN_Init(board_data.can_id);
    
    float current_bandwidth = 100.0; float sample_frequency = (1000000 / 50); float Tf_ratio = 5.0; 
    float vel_bandwidth_ratio = 0.1; float vel_PI_ratio = 10.0; float pos_bandwidth_ratio = 0.1; float pos_PI_ratio = 10.0;

    M1.velocity_limit = 99999; M1.voltage_limit = 1.0; M1.current_limit = 2.0;
    DR1.pwm_frequency = 20000; DR1.voltage_power_supply = 8.0; M1.voltage_sensor_align = 1.0;

    M1.motion_downsample = 9;

    M1.PID_current_q.P = q_phase_inductance*current_bandwidth*_2PI; M1.PID_current_q.I= M1.PID_current_q.P*RPHASE/q_phase_inductance;
    M1.PID_current_d.P= d_phase_inductance*current_bandwidth*_2PI; M1.PID_current_d.I = M1.PID_current_d.P*RPHASE/d_phase_inductance;
    M1.LPF_current_q.Tf = 1/(Tf_ratio*current_bandwidth); M1.LPF_current_d.Tf = 1/(Tf_ratio*current_bandwidth);
    M1.PID_velocity.P = M1.PID_current_q.P * vel_bandwidth_ratio; M1.PID_velocity.I = M1.PID_velocity.P*vel_PI_ratio;
    M1.LPF_velocity.Tf = (1/(Tf_ratio*M1.PID_velocity.P*sample_frequency)); 
    M1.P_angle.P = M1.PID_velocity.P*pos_bandwidth_ratio*sample_frequency; M1.P_angle.I = (M1.P_angle.P*pos_PI_ratio)/sample_frequency;
    
    E1.init(); M1.linkSensor(&E1); DR1.init(); M1.linkDriver(&DR1); CS1.linkDriver(&DR1); CS1.init(); M1.linkCurrentSense(&CS1);
    M1.foc_modulation = FOCModulationType::SpaceVectorPWM; M1.controller = MotionControlType::angle; M1.torque_controller = TorqueControlType::foc_current;
    
    M1.init();
    if (!board_data.encoder_calibrated) { runCalibrationSequence(); } 
    else { M1.sensor_direction = board_data.sensor_direction; M1.zero_electric_angle = board_data.zero_electric_angle; }
    M1.initFOC();
    M1.disable(); 
    SIMPLEFOC_DEBUG("Setup complete. Motor ready.");
    digitalWrite(LED_FAULT, LOW);
}

void loop() {
    unsigned long start_time_us = micros();

    M1.loopFOC();
    if (motor_enabled) { M1.move(target_value); }

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (CAN_Poll(&rxHeader, rxData)) {
        if (rxHeader.Identifier == board_data.can_id) {
            SimpleFOCRegister reg = (SimpleFOCRegister)rxData[0];
            
            if (reg == CAN_ID_REQUEST_PARAM) {
                uint8_t param_to_send = rxData[1];
                if (param_to_send == REG_VEL_PID_P) send_param_response(param_to_send, M1.PID_velocity.P);
                if (param_to_send == REG_VEL_PID_I) send_param_response(param_to_send, M1.PID_velocity.I);
                if (param_to_send == REG_ANG_PID_P) send_param_response(param_to_send, M1.P_angle.P);
                if (param_to_send == REG_ANG_PID_I) send_param_response(param_to_send, M1.P_angle.I);
                if (param_to_send == REG_CURQ_PID_P) send_param_response(param_to_send, M1.PID_current_q.P);
                if (param_to_send == REG_CURQ_PID_I) send_param_response(param_to_send, M1.PID_current_q.I);
                if (param_to_send == REG_CURD_PID_P) send_param_response(param_to_send, M1.PID_current_d.P);
                if (param_to_send == REG_CURD_PID_I) send_param_response(param_to_send, M1.PID_current_d.I);
                return;
            }

            switch(reg) {
                case REG_ENABLE:
                    motor_enabled = (rxData[1] == 1);
                    if (motor_enabled) M1.enable(); else M1.disable();
                    break;
                case REG_CONTROL_MODE:
                    M1.controller = (MotionControlType)rxData[1];
                    target_value = 0;
                    break;
                case REG_TELEMETRY_CONFIG:
                    if(rxHeader.DataLength >= 2) {
                        telemetry_mask = rxData[1];
                    }
                    break;
                case REG_TARGET:
                case REG_VEL_PID_P: case REG_VEL_PID_I:
                case REG_ANG_PID_P: case REG_ANG_PID_I:
                case REG_CURQ_PID_P: case REG_CURQ_PID_I:
                case REG_CURD_PID_P: case REG_CURD_PID_I: {
                    float payload_float;
                    if (rxHeader.DataLength >= 5) {
                        memcpy(&payload_float, &rxData[1], sizeof(float));
                        if(reg == REG_TARGET) target_value = payload_float;
                        if(reg == REG_VEL_PID_P) M1.PID_velocity.P = payload_float;
                        if(reg == REG_VEL_PID_I) M1.PID_velocity.I = payload_float;
                        if(reg == REG_ANG_PID_P) M1.P_angle.P = payload_float;
                        if(reg == REG_ANG_PID_I) M1.P_angle.I = payload_float;
                        if(reg == REG_CURQ_PID_P) M1.PID_current_q.P = payload_float;
                        if(reg == REG_CURQ_PID_I) M1.PID_current_q.I = payload_float;
                        if(reg == REG_CURD_PID_P) M1.PID_current_d.P = payload_float;
                        if(reg == REG_CURD_PID_I) M1.PID_current_d.I = payload_float;
                    }
                    break;
                }
                case REG_TELEMETRY_PERIOD: {
                    if (rxHeader.DataLength >= 3) {
                        uint16_t new_period;
                        memcpy(&new_period, &rxData[1], sizeof(uint16_t));
                        if (new_period > 0) telemetry_period = new_period;
                    }
                    break;
                }
            }
        }
    }

    if (millis() - last_telemetry_time > telemetry_period) {
        last_telemetry_time = millis();
        
        int states_checked = 0;
        while(states_checked < 4) {
            bool should_send = false;
            switch(telemetry_state) {
                case 0: if (telemetry_mask & TELEM_ENABLE_ANGLE) should_send = true; break;
                case 1: if (telemetry_mask & TELEM_ENABLE_VELOCITY) should_send = true; break;
                case 2: if (telemetry_mask & TELEM_ENABLE_CURRENT_Q) should_send = true; break;
                case 3: if (telemetry_mask & TELEM_ENABLE_LOOP_TIME) should_send = true; break;
            }

            if (should_send) {
                uint8_t telemetry_data[5];
                switch(telemetry_state) {
                    case 0: { telemetry_data[0] = TELEM_ANGLE; float v = M1.shaft_angle; memcpy(&telemetry_data[1], &v, 4); break; }
                    case 1: { telemetry_data[0] = TELEM_VELOCITY; float v = M1.shaft_velocity; memcpy(&telemetry_data[1], &v, 4); break; }
                    case 2: { telemetry_data[0] = TELEM_CURRENT_Q; float v = M1.current.q; memcpy(&telemetry_data[1], &v, 4); break; }
                    case 3: { telemetry_data[0] = TELEM_LOOP_TIME; memcpy(&telemetry_data[1], &loop_duration_us, 4); break; }
                }
                CAN_Send(CAN_ID_MOTOR_1_TELEMETRY, telemetry_data, FDCAN_DLC_BYTES_5);
                break;
            }

            telemetry_state = (telemetry_state + 1) % 4;
            states_checked++;
        }
        telemetry_state = (telemetry_state + 1) % 4;
    }

    loop_duration_us = micros() - start_time_us;
}

void send_param_response(uint8_t param_id, float value) {
    uint8_t response_data[5];
    response_data[0] = param_id;
    memcpy(&response_data[1], &value, sizeof(float));
    CAN_Send(CAN_ID_REGISTER_RESPONSE, response_data, FDCAN_DLC_BYTES_5);
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
        board_data.can_id = 1;
        saveSettings();
    }
}

void saveSettings() {
    EEPROM.put(0, board_data);
}