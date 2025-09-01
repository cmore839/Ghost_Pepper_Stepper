// main.cpp
#include <Arduino.h>
#include <EEPROM.h>
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "InlineCurrentSenseSync.h"
#include "can.h"

// --- Hardware Definitions ---
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

// --- SimpleFOC Objects ---
BLDCDriver3PWM DR1 = BLDCDriver3PWM(MOT_A1, MOT_A2, MOT_B1, MOT_EN);
BLDCMotor M1 = BLDCMotor(7);
SPISettings myMT6835SPISettings(12000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 E1 = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
InlineCurrentSenseSync CS1 = InlineCurrentSenseSync(0.005f, 50.0f, ISENSE_V, ISENSE_U, ISENSE_W);
PhaseCurrent_s current1;
float raw_current_A = 0;
float raw_current_B = 0;
float raw_current_C = 0;

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
const uint16_t BOARD_CONFIG_SIGNATURE = 0xBEED;

// --- Global Variables ---
uint32_t last_telemetry_time_us = 0;
uint32_t telemetry_period_us = 10000;
uint32_t last_status_time_us = 0;
uint32_t status_period_us = 50000; // 20Hz

// --- State Machine ---
enum MotorState {
    INITIALIZING,
    READY,
    OPERATIONAL,
    FAULT
};
MotorState current_state = INITIALIZING;

// --- Motion Command Buffer ---
struct MotionCommand {
    float position;
    float velocity;
    float acceleration;
};
MotionCommand motion_command_buffer;
volatile bool new_motion_command = false;

// --- Feedforward Gains ---
float K_ff_vel = 1.0;
float K_ff_accel = 0.0; 


// --- Function Prototypes ---
void sendPackedTelemetry();
void loadBoardConfig();
void saveBoardConfig();
void applySettingsToMotor();
void runCalibrationSequence();
void send_param_response(uint8_t param_id, float value);
void send_param_response(uint8_t param_id, uint8_t value);
void send_characterization_response(float resistance, float inductance);
void update_config_from_motor();
void runMotorCharacterization(float voltage);
void sendStatusFeedback();


void setup() {
    pinMode(LED_FAULT, OUTPUT);
    digitalWrite(LED_FAULT, HIGH);
    
    Serial3.begin(115200);
    SimpleFOCDebug::enable(&Serial3);
    SIMPLEFOC_DEBUG("--- Robust Custom CAN Firmware ---");

    loadBoardConfig();
    CAN_Init(board_config.can_id);
    
    applySettingsToMotor();

    E1.init(); 
    DR1.init();
    M1.linkSensor(&E1);
    M1.linkDriver(&DR1);

    SIMPLEFOC_DEBUG("Calibrating current sense...");
    CS1.linkDriver(&DR1);
    if(CS1.init() != 1) { 
        SIMPLEFOC_DEBUG("ERR: CS init failed!");
        current_state = FAULT;
        while(1);
    }
    M1.linkCurrentSense(&CS1);
    SIMPLEFOC_DEBUG("...done.");   
    
    M1.foc_modulation = FOCModulationType::SpaceVectorPWM; 
    M1.torque_controller = TorqueControlType::foc_current; 
    
    M1.init();

    if (!board_config.encoder_calibrated) {
        runCalibrationSequence();
    }
    
    M1.initFOC();
    M1.disable(); 
    Serial3.print("Setup complete. Motor ready with CAN ID: ");
    Serial3.println(board_config.can_id);
    digitalWrite(LED_FAULT, LOW);
    
    current_state = READY;
}

void loop() {
    if (current_state == FAULT) {
        digitalWrite(LED_FAULT, HIGH);
        return;
    }

    E1.update();
   
    M1.move();
    M1.loopFOC();

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    
    if (CAN_Poll(&rxHeader, &rxData[0])) {
        if (rxHeader.Identifier == CAN_ID_SCAN_BROADCAST) {
            sendPackedTelemetry();
        } else if (rxHeader.Identifier == (CAN_ID_MOTION_COMMAND_BASE + board_config.can_id)) {
            if (rxHeader.DataLength >= 8) { 
                int32_t pos_raw;
                // --- FIX: Revert vel_raw and acc_raw to 16-bit integers ---
                int16_t vel_raw;
                int16_t acc_raw;
                
                // Unpack the 8-byte message
                memcpy(&pos_raw, &rxData[0], sizeof(int32_t));
                memcpy(&vel_raw, &rxData[4], sizeof(int16_t)); 
                memcpy(&acc_raw, &rxData[6], sizeof(int16_t));
                
                // --- FIX: Use the new corresponding scaling factors ---
                motion_command_buffer.position = pos_raw * 0.0001f; 
                motion_command_buffer.velocity = vel_raw * 0.01f;     // Changed from 0.001
                motion_command_buffer.acceleration = acc_raw * 0.1f;  // Changed from 0.01
                
                new_motion_command = true;
            }
        } else if (rxHeader.Identifier == CAN_ID_SYNC) {
            Serial3.printf("SYNC received. Motor enabled: %d, State: %d", M1.enabled, current_state);
            // SYNC now simply means "make sure you're running"
            if (M1.enabled && current_state == READY) {
                M1.controller = MotionControlType::angle;
                current_state = OPERATIONAL;
                SIMPLEFOC_DEBUG("State -> OPERATIONAL");
            }
        }
        else if (rxHeader.Identifier == (CAN_ID_COMMAND_BASE + board_config.can_id)) {
            SimpleFOCRegister reg = (SimpleFOCRegister)rxData[0];
            
            if (rxHeader.DataLength == 1 && reg < 0xE0) { 
                float val_f = 0.0f;
                switch(reg) {
                    case REG_STATUS: send_param_response(reg, (uint8_t)M1.enabled); break;
                    case REG_VEL_PID_P: val_f = M1.PID_velocity.P; send_param_response(reg, val_f); break;
                    case REG_VEL_PID_I: val_f = M1.PID_velocity.I; send_param_response(reg, val_f); break;
                    case REG_VEL_PID_D: val_f = M1.PID_velocity.D; send_param_response(reg, val_f); break;
                    case REG_VEL_PID_LIM: val_f = M1.PID_velocity.limit; send_param_response(reg, val_f); break;
                    case REG_VEL_PID_RAMP: val_f = M1.PID_velocity.output_ramp; send_param_response(reg, val_f); break;
                    case REG_VEL_LPF_T: val_f = M1.LPF_velocity.Tf; send_param_response(reg, val_f); break;
                    case REG_ANG_PID_P: val_f = M1.P_angle.P; send_param_response(reg, val_f); break;
                    case REG_ANG_PID_I: val_f = M1.P_angle.I; send_param_response(reg, val_f); break;
                    case REG_ANG_PID_D: val_f = M1.P_angle.D; send_param_response(reg, val_f); break;
                    case REG_ANG_PID_LIM: val_f = M1.P_angle.limit; send_param_response(reg, val_f); break;
                    case REG_ANG_PID_RAMP: val_f = M1.P_angle.output_ramp; send_param_response(reg, val_f); break;
                    case REG_CURQ_PID_P: val_f = M1.PID_current_q.P; send_param_response(reg, val_f); break;
                    case REG_CURQ_PID_I: val_f = M1.PID_current_q.I; send_param_response(reg, val_f); break;
                    case REG_CURQ_PID_D: val_f = M1.PID_current_q.D; send_param_response(reg, val_f); break;
                    case REG_CURQ_LPF_T: val_f = M1.LPF_current_q.Tf; send_param_response(reg, val_f); break;
                    case REG_CURD_PID_P: val_f = M1.PID_current_d.P; send_param_response(reg, val_f); break;
                    case REG_CURD_PID_I: val_f = M1.PID_current_d.I; send_param_response(reg, val_f); break;
                    case REG_CURD_PID_D: val_f = M1.PID_current_d.D; send_param_response(reg, val_f); break;
                    case REG_CURD_LPF_T: val_f = M1.LPF_current_d.Tf; send_param_response(reg, val_f); break;
                    case REG_VOLTAGE_LIMIT: val_f = M1.voltage_limit; send_param_response(reg, val_f); break;
                    case REG_CURRENT_LIMIT: val_f = M1.current_limit; send_param_response(reg, val_f); break;
                    case REG_VELOCITY_LIMIT: val_f = M1.velocity_limit; send_param_response(reg, val_f); break;
                    case REG_DRIVER_VOLTAGE_PSU: val_f = DR1.voltage_power_supply; send_param_response(reg, val_f); break;
                    case REG_VOLTAGE_SENSOR_ALIGN: val_f = M1.voltage_sensor_align; send_param_response(reg, val_f); break;
                    case REG_POLE_PAIRS: val_f = (float)M1.pole_pairs; send_param_response(reg, val_f); break;
                    case REG_PHASE_RESISTANCE: val_f = M1.phase_resistance; send_param_response(reg, val_f); break;
                    case REG_INDUCTANCE: val_f = M1.phase_inductance; send_param_response(reg, val_f); break;
                    case REG_KV: val_f = M1.KV_rating; send_param_response(reg, val_f); break;
                }
            } 
            else { // WRITE request
                float val_f; uint32_t val_32; uint8_t val_8 = rxData[1];
                if (rxHeader.DataLength >= 5) memcpy(&val_f, &rxData[1], sizeof(float));
                if (rxHeader.DataLength >= 5) memcpy(&val_32, &rxData[1], sizeof(uint32_t));

                switch(reg) {
                    case REG_TARGET: 
                        // For manual control, set the target and zero out the feedforward terms
                        M1.target = val_f;
                        M1.velocity_ff = 0.0f;
                        M1.acceleration_ff = 0.0f;
                        break;
                    case REG_ENABLE: 
                        if (val_8 > 0) {
                            M1.enable();
                            current_state = READY;
                            SIMPLEFOC_DEBUG("Motor ENABLED. State -> READY");
                        } else {
                            M1.disable();
                            current_state = READY;
                            SIMPLEFOC_DEBUG("Motor DISABLED. State -> READY");
                        }
                        break;
                    case REG_CONTROL_MODE: M1.controller = (MotionControlType)val_8; break;
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
                    case REG_DRIVER_VOLTAGE_PSU: DR1.voltage_power_supply = val_f; break;
                    case REG_VOLTAGE_SENSOR_ALIGN: M1.voltage_sensor_align = val_f; break;
                    case REG_POLE_PAIRS: M1.pole_pairs = val_8; break;
                    case REG_PHASE_RESISTANCE: M1.phase_resistance = val_f; break;
                    case REG_INDUCTANCE: M1.phase_inductance = val_f; break;
                    case REG_KV: M1.KV_rating = val_f; break;
                    
                    // Custom Commands
                    case REG_CUSTOM_SAVE_TO_EEPROM: 
                        SIMPLEFOC_DEBUG("Save command received.");
                        update_config_from_motor();
                        saveBoardConfig();
                        break;
                    case REG_CUSTOM_FLIP_SENSOR_DIR:
                        M1.disable();
                        board_config.sensor_direction = (board_config.sensor_direction == Direction::CW) ? Direction::CCW : Direction::CW;
                        board_config.zero_electric_angle = _2PI - board_config.zero_electric_angle;
                        update_config_from_motor();
                        saveBoardConfig();
                        M1.initFOC();
                        break;
                    case REG_CUSTOM_TELEMETRY_PERIOD: telemetry_period_us = val_32; break;
                    case REG_CUSTOM_CHARACTERIZE_MOTOR:
                        runMotorCharacterization(val_f); 
                        send_characterization_response(M1.phase_resistance, M1.phase_inductance);
                        break;
                    case REG_CUSTOM_SET_ID_AND_RESTART:
                        if (rxHeader.DataLength == 2) {
                            uint8_t new_id = rxData[1];
                            Serial3.print("Received command to set new ID to ");
                            Serial3.print(new_id);
                            Serial3.println(" and restart.");
                            board_config.can_id = new_id;
                            update_config_from_motor(); 
                            saveBoardConfig();          
                            Serial3.println("Config saved. Restarting MCU now.");
                            delay(100);                 
                            NVIC_SystemReset();
                        }
                        break;
                }
            }
        }
    }

    // ================== FIX STARTS HERE ==================
    // This is the missing logic block to process the motion command buffer.
    if (new_motion_command) {
        // Only apply the new command if the motor is in the OPERATIONAL state
        if (current_state == OPERATIONAL) {
            M1.target = motion_command_buffer.position;
            M1.velocity_ff = K_ff_vel * motion_command_buffer.velocity;
            M1.acceleration_ff = K_ff_accel * motion_command_buffer.acceleration;
        }
        // Reset the flag so this command isn't processed again
        new_motion_command = false;
    }
    // =================== FIX ENDS HERE ===================

    unsigned long now = micros();
    if (telemetry_period_us > 0 && (now - last_telemetry_time_us > telemetry_period_us)) {
        last_telemetry_time_us = now;
        sendPackedTelemetry();
    }
    if (status_period_us > 0 && (now - last_status_time_us > status_period_us)) {
        last_status_time_us = now;
        sendStatusFeedback();
    }
}

void runMotorCharacterization(float voltage) {
    const float CURRENT_SENSE_CORRECTION_FACTOR = 1.0f; 

    if (!CS1.initialized) {
        Serial3.println(F("ERR: MOT: Cannot characterise motor: CS not initialized"));
        current_state = FAULT;
        return;
    }

    if (voltage <= 0.0f){
        Serial3.println(F("ERR: MOT: Voltage is negative or less than zero"));
        return;
    }
    voltage = _constrain(voltage, 0.0f, M1.voltage_limit);
    
    Serial3.println(F("MOT: Measuring phase resistance..."));

    // 1. Measure zero-current offset
    DR1.setPwm(0, 0, 0);
    DR1.disable();
    _delay(2000); 
    float current_offset_B = 0;
    for (int i = 0; i < 100; i++) {
        current_offset_B += CS1.getPhaseCurrents().b;
        _delay(1);
    }
    current_offset_B /= 100.0f;

    Serial3.print(F("MOT-DEBUG: Measured current offset B: "));
    Serial3.print(current_offset_B, 4);
    Serial3.println(F(" A"));

    // 2. Apply voltage and measure current    
    DR1.enable();
    DR1.setPwm(0, voltage, -voltage); 
    _delay(1000);
     
    for (int i = 0; i < 100; i++) {
        raw_current_A += CS1.getPhaseCurrents().a;
        raw_current_B += CS1.getPhaseCurrents().b;
        raw_current_C += CS1.getPhaseCurrents().c;
        _delay(5);
    }
    raw_current_C /= 100.0f;
    raw_current_B /= 100.0f;
    raw_current_A /= 100.0f;
    _delay(1000);
    
    DR1.setPwm(0, 0, 0);
    _delay(200);
    DR1.disable();

    Serial3.print(F("MOT-DEBUG: Raw current B under load: "));
    Serial3.print(raw_current_B, 4);
    Serial3.println(F(" A"));

    // 3. Calculate resistance
    float true_current_B = (raw_current_B - current_offset_B) * CURRENT_SENSE_CORRECTION_FACTOR;
    
    Serial3.print(F("MOT-DEBUG: Corrected current B: "));
    Serial3.print(true_current_B, 4);
    Serial3.println(F(" A"));
    
    if (fabsf(true_current_B) < 0.01f) { 
        Serial3.println(F("ERR: MOT: Motor characterisation failed: corrected current too low"));
        current_state = FAULT;
        return;
    }
    
    float terminal_resistance = voltage / true_current_B;
    M1.phase_resistance = terminal_resistance / 2.0f;
    
    Serial3.print(F("MOT: Estimated phase resistance: "));
    Serial3.print(M1.phase_resistance, 4);
    Serial3.println(F(" Ohms"));
    _delay(100);

    Serial3.println(F("MOT: Measuring phase inductance..."));
    long t_start = _micros();
    DR1.enable();
    DR1.setPwm(0, voltage, 0);
    while(_micros() - t_start < 1000000) { 
        PhaseCurrent_s current_check = CS1.getPhaseCurrents();
        if (abs((current_check.b - current_offset_B) * CURRENT_SENSE_CORRECTION_FACTOR) > 0.632f * (voltage / terminal_resistance)) {
            break;
        }
    }
    long t_end = _micros();
    DR1.setPwm(0, 0, 0);
    DR1.disable();

    float time_constant = (t_end - t_start) * 1e-6f;
    M1.phase_inductance = terminal_resistance * time_constant;

    Serial3.print(F("MOT: Measured L: "));
    Serial3.print(M1.phase_inductance, 6);
    Serial3.println(F(" H"));

    Serial3.println(F("MOT: Characterization complete."));
    raw_current_A = 0;
    raw_current_B = 0;
    raw_current_C = 0;
}

void update_config_from_motor() {
    SIMPLEFOC_DEBUG("Syncing motor state to config struct...");
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
}

void saveBoardConfig(){
    Serial3.print("Saving config to EEPROM. CAN ID will be: ");
    Serial3.println(board_config.can_id);
    EEPROM.put(0, board_config);
    delay(20); 
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
        board_config.phase_resistance = 0.0f;
        board_config.phase_inductance = 0.0f;
        board_config.kv_rating = 0.0f;
        board_config.voltage_limit = 1.0f;
        board_config.current_limit = 1.0f;
        board_config.velocity_limit = 50.0f;
        board_config.driver_voltage_psu = 8.0f;
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
    uint8_t tx_data[8];
    int32_t angle_raw = (int32_t)(M1.shaft_angle * 10000.0f);
    memcpy(&tx_data[0], &angle_raw, sizeof(int32_t));
    int16_t velocity_raw = (int16_t)(M1.shaft_velocity * 100.0f);
    memcpy(&tx_data[4], &velocity_raw, sizeof(int16_t));
    int16_t current_q_raw = (int16_t)(M1.current.q * 1000.0f);
    memcpy(&tx_data[6], &current_q_raw, sizeof(int16_t));
    CAN_Send(CAN_ID_TELEMETRY_BASE + board_config.can_id, tx_data, 8);
}

void send_param_response(uint8_t param_id, float value) {
    uint8_t response_data[5];
    response_data[0] = param_id;
    memcpy(&response_data[1], &value, sizeof(float));
    CAN_Send(CAN_ID_RESPONSE_BASE + board_config.can_id, response_data, 5);
}

void send_param_response(uint8_t param_id, uint8_t value) {
    uint8_t response_data[2];
    response_data[0] = param_id;
    response_data[1] = value;
    CAN_Send(CAN_ID_RESPONSE_BASE + board_config.can_id, response_data, 2);
}

void send_characterization_response(float resistance, float inductance) {
    uint8_t response_data[8];
    memcpy(&response_data[0], &resistance, sizeof(float));
    memcpy(&response_data[4], &inductance, sizeof(float));
    CAN_Send(CAN_ID_RESPONSE_BASE + board_config.can_id + 0x80, response_data, 8); 
}

void sendStatusFeedback() {
    uint8_t tx_data[8];
    int32_t angle_raw = (int32_t)(M1.shaft_angle * 10000);
    memcpy(&tx_data[0], &angle_raw, sizeof(int32_t));
    int16_t velocity_raw = (int16_t)(M1.shaft_velocity * 100);
    memcpy(&tx_data[4], &velocity_raw, sizeof(int16_t));
    uint8_t status_flags = 0;
    if (M1.enabled) status_flags |= 0x01;
    if (current_state == FAULT) status_flags |= 0x02;
    tx_data[6] = status_flags;
    tx_data[7] = (uint8_t)current_state;
    CAN_Send(CAN_ID_STATUS_FEEDBACK_BASE + board_config.can_id, tx_data, 8);
}

void runCalibrationSequence() {
    SIMPLEFOC_DEBUG("Starting calibration...");
    M1.zero_electric_angle = NOT_SET;
    M1.sensor_direction = Direction::UNKNOWN;
    M1.initFOC();
    board_config.encoder_calibrated = 1;
    update_config_from_motor();
    saveBoardConfig();
    SIMPLEFOC_DEBUG("Calibration complete.");
}