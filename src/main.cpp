// main.cpp
#include <Arduino.h>
#include <EEPROM.h>
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "InlineCurrentSenseSync.h"
#include "can.h"

// --- Hardware Definitions ---
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

// SimpleFOC Objects
BLDCDriver3PWM DR1 = BLDCDriver3PWM(MOT_A1, MOT_A2, MOT_B1, MOT_EN);
BLDCMotor M1 = BLDCMotor(7);
SPISettings myMT6835SPISettings(12000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 E1 = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
InlineCurrentSenseSync CS1 = InlineCurrentSenseSync(0.005, 50, ISENSE_V, ISENSE_U, ISENSE_W);

// --- EEPROM Data Structure ---
typedef struct {
    uint16_t signature;
    uint8_t can_id;
    uint8_t encoder_calibrated;
    Direction sensor_direction;
    float zero_electric_angle;
    uint8_t pole_pairs;
    float phase_resistance;
    float phase_inductance;
    float kv_rating;
    float voltage_limit;
    float current_limit;
    float velocity_limit;
    float driver_voltage_psu;
    float voltage_sensor_align;
    float vel_p, vel_i, vel_d, vel_lim, vel_ramp, vel_lpf_t;
    float ang_p, ang_i, ang_d, ang_lim, ang_ramp;
    float curq_p, curq_i, curq_d, curq_lpf_t;
    float curd_p, curd_i, curd_d, curd_lpf_t;
} BoardSettings;
BoardSettings board_config;
const uint16_t BOARD_CONFIG_SIGNATURE = 0xBEEF;

// --- Global Variables ---
uint32_t last_telemetry_time_us = 0;
uint32_t telemetry_period_us = 10000;

// --- Function Prototypes ---
void sendPackedTelemetry();
void loadBoardConfig();
void saveBoardConfig();
void applySettingsToMotor();
void runCalibrationSequence();
void send_param_response(uint8_t param_id, float value);

void setup() {
    pinMode(LED_FAULT, OUTPUT);
    digitalWrite(LED_FAULT, HIGH);
    
    Serial3.begin(115200);
    SimpleFOCDebug::enable(&Serial3);
    SIMPLEFOC_DEBUG("--- Robust Custom CAN Firmware ---");

    loadBoardConfig();
    CAN_Init(board_config.can_id);
    
    E1.init(); M1.linkSensor(&E1); DR1.init(); M1.linkDriver(&DR1); CS1.linkDriver(&DR1); CS1.init(); M1.linkCurrentSense(&CS1);
    
    applySettingsToMotor();
    
    M1.foc_modulation = FOCModulationType::SpaceVectorPWM; 
    M1.torque_controller = TorqueControlType::foc_current;
    
    M1.init();
    
    if (!board_config.encoder_calibrated) {
        runCalibrationSequence();
    }
    
    M1.initFOC();
    M1.disable(); 
    SIMPLEFOC_DEBUG("Setup complete. Motor ready.");
    digitalWrite(LED_FAULT, LOW);
}

void loop() {
    M1.loopFOC();
    M1.move();

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    
    if (CAN_Poll(&rxHeader, rxData)) {
        if (rxHeader.Identifier == CAN_ID_SCAN_BROADCAST) {
            sendPackedTelemetry();
        }
        else if (rxHeader.Identifier == (CAN_ID_COMMAND_BASE + board_config.can_id)) {
            SimpleFOCRegister reg = (SimpleFOCRegister)rxData[0];
            
            if (rxHeader.DataLength == 1) { // READ request
                float val = 0.0f;
                switch(reg) {
                    case REG_VEL_PID_P: val = M1.PID_velocity.P; break;
                    case REG_VEL_PID_I: val = M1.PID_velocity.I; break;
                    case REG_VEL_PID_D: val = M1.PID_velocity.D; break;
                    case REG_VEL_PID_LIM: val = M1.PID_velocity.limit; break;
                    case REG_VEL_PID_RAMP: val = M1.PID_velocity.output_ramp; break;
                    case REG_VEL_LPF_T: val = M1.LPF_velocity.Tf; break;
                    case REG_ANG_PID_P: val = M1.P_angle.P; break;
                    case REG_ANG_PID_I: val = M1.P_angle.I; break;
                    case REG_ANG_PID_D: val = M1.P_angle.D; break;
                    case REG_ANG_PID_LIM: val = M1.P_angle.limit; break;
                    case REG_ANG_PID_RAMP: val = M1.P_angle.output_ramp; break;
                    case REG_CURQ_PID_P: val = M1.PID_current_q.P; break;
                    case REG_CURQ_PID_I: val = M1.PID_current_q.I; break;
                    case REG_CURQ_PID_D: val = M1.PID_current_q.D; break;
                    case REG_CURQ_LPF_T: val = M1.LPF_current_q.Tf; break;
                    case REG_CURD_PID_P: val = M1.PID_current_d.P; break;
                    case REG_CURD_PID_I: val = M1.PID_current_d.I; break;
                    case REG_CURD_PID_D: val = M1.PID_current_d.D; break;
                    case REG_CURD_LPF_T: val = M1.LPF_current_d.Tf; break;
                    case REG_VOLTAGE_LIMIT: val = M1.voltage_limit; break;
                    case REG_CURRENT_LIMIT: val = M1.current_limit; break;
                    case REG_VELOCITY_LIMIT: val = M1.velocity_limit; break;
                    case REG_DRIVER_VOLTAGE_PSU: val = M1.driver->voltage_power_supply; break;
                    case REG_VOLTAGE_SENSOR_ALIGN: val = M1.voltage_sensor_align; break;
                    case REG_POLE_PAIRS: val = (float)M1.pole_pairs; break;
                    case REG_PHASE_RESISTANCE: val = M1.phase_resistance; break;
                    case REG_INDUCTANCE: val = M1.phase_inductance; break;
                    case REG_KV: val = M1.KV_rating; break;
                }
                send_param_response(reg, val);
            } 
            else { // WRITE request
                float val_f; uint32_t val_32; uint8_t val_8 = rxData[1];
                if (rxHeader.DataLength >= 5) memcpy(&val_f, &rxData[1], sizeof(float));
                if (rxHeader.DataLength >= 5) memcpy(&val_32, &rxData[1], sizeof(uint32_t));

                switch(reg) {
                    case REG_TARGET: M1.target = val_f; break;
                    case REG_ENABLE: (val_8 > 0) ? M1.enable() : M1.disable(); break;
                    case REG_CONTROL_MODE: M1.controller = (MotionControlType)val_8; break;
                    case REG_MOTOR_ADDRESS: board_config.can_id = val_8; CAN_Init(val_8); break;
                    case REG_VEL_PID_P: M1.PID_velocity.P = val_f; break;
                    case REG_VEL_PID_I: M1.PID_velocity.I = val_f; break;
                    case REG_VEL_PID_D: M1.PID_velocity.D = val_f; break;
                    case REG_VEL_PID_LIM: M1.PID_velocity.limit = val_f; break;
                    case REG_VEL_PID_RAMP: M1.PID_velocity.output_ramp = val_f; break;
                    case REG_VEL_LPF_T: M1.LPF_velocity.Tf = val_f; break;
                    case REG_ANG_PID_P: M1.P_angle.P = val_f; break;
                    case REG_ANG_PID_I: M1.P_angle.I = val_f; break;
                    case REG_ANG_PID_D: M1.P_angle.D = val_f; break;
                    case REG_ANG_PID_LIM: M1.P_angle.limit = val_f; break;
                    case REG_ANG_PID_RAMP: M1.P_angle.output_ramp = val_f; break;
                    case REG_CURQ_PID_P: M1.PID_current_q.P = val_f; break;
                    case REG_CURQ_PID_I: M1.PID_current_q.I = val_f; break;
                    case REG_CURQ_PID_D: M1.PID_current_q.D = val_f; break;
                    case REG_CURQ_LPF_T: M1.LPF_current_q.Tf = val_f; break;
                    case REG_CURD_PID_P: M1.PID_current_d.P = val_f; break;
                    case REG_CURD_PID_I: M1.PID_current_d.I = val_f; break;
                    case REG_CURD_PID_D: M1.PID_current_d.D = val_f; break;
                    case REG_CURD_LPF_T: M1.LPF_current_d.Tf = val_f; break;
                    case REG_VOLTAGE_LIMIT: M1.voltage_limit = val_f; break;
                    case REG_CURRENT_LIMIT: M1.current_limit = val_f; break;
                    case REG_VELOCITY_LIMIT: M1.velocity_limit = val_f; break;
                    case REG_DRIVER_VOLTAGE_PSU: M1.driver->voltage_power_supply = val_f; break;
                    case REG_VOLTAGE_SENSOR_ALIGN: M1.voltage_sensor_align = val_f; break;
                    case REG_POLE_PAIRS: M1.pole_pairs = val_8; break;
                    case REG_PHASE_RESISTANCE: M1.phase_resistance = val_f; break;
                    case REG_INDUCTANCE: M1.phase_inductance = val_f; break;
                    case REG_KV: M1.KV_rating = val_f; break;
                    // Custom Commands
                    case REG_CUSTOM_SAVE_TO_EEPROM: saveBoardConfig(); break;
                    case REG_CUSTOM_FLIP_SENSOR_DIR:
                        M1.disable();
                        board_config.sensor_direction = (board_config.sensor_direction == Direction::CW) ? Direction::CCW : Direction::CW;
                        board_config.zero_electric_angle = _2PI - board_config.zero_electric_angle;
                        M1.sensor_direction = board_config.sensor_direction;
                        M1.zero_electric_angle = board_config.zero_electric_angle;
                        M1.initFOC();
                        saveBoardConfig();
                        break;
                    case REG_CUSTOM_TELEMETRY_PERIOD: telemetry_period_us = val_32; break;
                }
            }
        }
    }

    unsigned long now = micros();
    if (telemetry_period_us > 0 && (now - last_telemetry_time_us > telemetry_period_us)) {
        last_telemetry_time_us = now;
        sendPackedTelemetry();
    }
}

void saveBoardConfig(){
    SIMPLEFOC_DEBUG("Saving all parameters to EEPROM...");
    board_config.sensor_direction = M1.sensor_direction;
    board_config.zero_electric_angle = M1.zero_electric_angle;
    board_config.pole_pairs = M1.pole_pairs;
    board_config.phase_resistance = M1.phase_resistance;
    board_config.phase_inductance = M1.phase_inductance;
    board_config.kv_rating = M1.KV_rating;
    board_config.voltage_limit = M1.voltage_limit;
    board_config.current_limit = M1.current_limit;
    board_config.velocity_limit = M1.velocity_limit;
    board_config.driver_voltage_psu = DR1.voltage_power_supply;
    board_config.voltage_sensor_align = M1.voltage_sensor_align;
    board_config.vel_p = M1.PID_velocity.P; board_config.vel_i = M1.PID_velocity.I; board_config.vel_d = M1.PID_velocity.D;
    board_config.vel_lim = M1.PID_velocity.limit; board_config.vel_ramp = M1.PID_velocity.output_ramp; board_config.vel_lpf_t = M1.LPF_velocity.Tf;
    board_config.ang_p = M1.P_angle.P; board_config.ang_i = M1.P_angle.I; board_config.ang_d = M1.P_angle.D;
    board_config.ang_lim = M1.P_angle.limit; board_config.ang_ramp = M1.P_angle.output_ramp;
    board_config.curq_p = M1.PID_current_q.P; board_config.curq_i = M1.PID_current_q.I; board_config.curq_d = M1.PID_current_q.D;
    board_config.curq_lpf_t = M1.LPF_current_q.Tf;
    board_config.curd_p = M1.PID_current_d.P; board_config.curd_i = M1.PID_current_d.I; board_config.curd_d = M1.PID_current_d.D;
    board_config.curd_lpf_t = M1.LPF_current_d.Tf;
    
    EEPROM.put(0, board_config);
    delay(10);
    SIMPLEFOC_DEBUG("Save complete.");
}

void loadBoardConfig(){
    EEPROM.get(0, board_config);
    if (board_config.signature != BOARD_CONFIG_SIGNATURE) {
        SIMPLEFOC_DEBUG("Board config not found. Initializing with defaults.");
        board_config.signature = BOARD_CONFIG_SIGNATURE;
        board_config.can_id = 1;
        board_config.encoder_calibrated = 0;
        board_config.pole_pairs = 7;
        board_config.phase_resistance = 0.6f;
        board_config.phase_inductance = 0.0015f;
        board_config.kv_rating = 360;
        board_config.voltage_limit = 8.0f;
        board_config.current_limit = 1.0f;
        board_config.velocity_limit = 50.0f;
        board_config.driver_voltage_psu = 12.0f;
        board_config.voltage_sensor_align = 2.0f;
        board_config.vel_p = 0.2f; board_config.vel_i = 2.0f; board_config.vel_d = 0.0f;
        board_config.vel_lim = 10.0f; board_config.vel_ramp = 1000.0f; board_config.vel_lpf_t = 0.01f;
        board_config.ang_p = 20.0f; board_config.ang_i = 0.0f; board_config.ang_d = 0.0f;
        board_config.ang_lim = 1000.0f; board_config.ang_ramp = 1000.0f;
        board_config.curq_p = 5.0f; board_config.curq_i = 1000.0f; board_config.curq_d = 0.0f;
        board_config.curq_lpf_t = 0.001f;
        board_config.curd_p = 5.0f; board_config.curd_i = 1000.0f; board_config.curd_d = 0.0f;
        board_config.curd_lpf_t = 0.001f;
        saveBoardConfig();
    }
}

void applySettingsToMotor() {
    M1.pole_pairs = board_config.pole_pairs;
    M1.phase_resistance = board_config.phase_resistance;
    M1.phase_inductance = board_config.phase_inductance;
    M1.KV_rating = board_config.kv_rating;
    M1.voltage_limit = board_config.voltage_limit;
    M1.current_limit = board_config.current_limit;
    M1.velocity_limit = board_config.velocity_limit;
    DR1.voltage_power_supply = board_config.driver_voltage_psu;
    M1.voltage_sensor_align = board_config.voltage_sensor_align;
    M1.PID_velocity.P = board_config.vel_p; M1.PID_velocity.I = board_config.vel_i; M1.PID_velocity.D = board_config.vel_d;
    M1.PID_velocity.limit = board_config.vel_lim; M1.PID_velocity.output_ramp = board_config.vel_ramp;
    M1.LPF_velocity.Tf = board_config.vel_lpf_t;
    M1.P_angle.P = board_config.ang_p; M1.P_angle.I = board_config.ang_i; M1.P_angle.D = board_config.ang_d;
    M1.P_angle.limit = board_config.ang_lim; M1.P_angle.output_ramp = board_config.ang_ramp;
    M1.PID_current_q.P = board_config.curq_p; M1.PID_current_q.I = board_config.curq_i; M1.PID_current_q.D = board_config.curq_d;
    M1.LPF_current_q.Tf = board_config.curq_lpf_t;
    M1.PID_current_d.P = board_config.curd_p; M1.PID_current_d.I = board_config.curd_i; M1.PID_current_d.D = board_config.curd_d;
    M1.LPF_current_d.Tf = board_config.curd_lpf_t;
    M1.sensor_direction = board_config.sensor_direction;
    M1.zero_electric_angle = board_config.zero_electric_angle;
}

void sendPackedTelemetry() {
    uint8_t tx_data[8] = {0};
    int32_t angle_raw = (int32_t)(M1.shaft_angle / 0.0001f);
    memcpy(&tx_data[0], &angle_raw, sizeof(int32_t));
    int16_t velocity_raw = (int16_t)(M1.shaft_velocity / 0.01f);
    memcpy(&tx_data[4], &velocity_raw, sizeof(int16_t));
    int16_t current_q_raw = (int16_t)(M1.current.q / 0.001f);
    memcpy(&tx_data[6], &current_q_raw, sizeof(int16_t));
    CAN_Send(CAN_ID_TELEMETRY_BASE + board_config.can_id, tx_data, FDCAN_DLC_BYTES_8);
}

void send_param_response(uint8_t param_id, float value) {
    uint8_t response_data[5] = {0};
    response_data[0] = param_id;
    memcpy(&response_data[1], &value, sizeof(float));
    CAN_Send(CAN_ID_RESPONSE_BASE + board_config.can_id, response_data, 5);
}

void runCalibrationSequence() {
    SIMPLEFOC_DEBUG("Starting calibration...");
    M1.initFOC();
    board_config.encoder_calibrated = 1;
    board_config.sensor_direction = M1.sensor_direction;
    board_config.zero_electric_angle = M1.zero_electric_angle;
    saveBoardConfig();
    SIMPLEFOC_DEBUG("Calibration complete.");
}