#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#include "can.h"
#include "dfu.h"
#include "utils.h"
#include "InlineCurrentSenseSync.h"

#define ENC_COPI PA7
#define ENC_CIPO PA6
#define ENC_CS PC4
#define ENC_SCK PA5
#define CAL_EN PA4

#define MOT_A1      PA0     // TIM2_CH1
#define MOT_A2      PA10    // TIM2_CH4
#define MOT_B1      PA9     // TIM2_CH3
#define MOT_B2      PA1     // TIM2_CH2

#define MOT_EN      PB12

#define ISENSE_V    PA3     // VOPAMP1_P, ADC1_IN4
#define ISENSE_U    PB13    // VOPAMP3_P
#define ISENSE_W    PB0 //NC

#define CAN_TX      PB9
#define CAN_RX      PB8

#define POLEPAIRS 7
#define RPHASE 0.6
#define MOTORKV 360
#define q_phase_inductance 0.0015 // H
#define d_phase_inductance 0.0015 // H
// Define the button pin
#define BUTTON_PIN PC15
#define LED_FAULT   PB11

bool motor_enabled = true;
bool last_button_state;
bool reading = 0;
unsigned long last_debounce_time = 0;
const unsigned long debounce_delay = 50; // ms
static bool button_was_pressed;

//VARS
PhaseCurrent_s current;
DQCurrent_s foc_currents;
float current_magnitude = 0; // current magnitude variable
float received_angle = 0; // angle set point variable
float loop_count = 0;
int start;
int finish;
int looptime;
int loopcounter = 0;
int loopiter = 10;
unsigned int timestamp = micros();

//Current Tune
float current_bandwidth = 100.0; // Hz
float sample_frequency = (1000000/50); // 50us loop =  20kHz 
float Tf_ratio = 5.0; // ratio of the low pass filter time constant to the current control bandwidth
float vel_bandwidth_ratio = 0.1;
float vel_PI_ratio = 10.0; // ratio of the velocity PI controller to the current controller
float pos_bandwidth_ratio = 0.1;
float pos_PI_ratio = 10.0; // ratio of the position PI controller to the velocity controller 

BLDCDriver3PWM DR1 = BLDCDriver3PWM(MOT_A1, MOT_A2, MOT_B1, MOT_EN);
BLDCMotor M1 = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
SPISettings myMT6835SPISettings(12000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 E1 = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
InlineCurrentSenseSync CS1  = InlineCurrentSenseSync(0.005, 50, ISENSE_V, ISENSE_U, ISENSE_W);

// board specific data
typedef struct
{
	uint16_t signature;
	Direction electricalDir;
	float electricalZero;
	uint16_t abzResolution;
	uint8_t encoderCalibrated;
	uint8_t canID;
} userData;

userData boardData;
uint8_t updateData = 0;

const uint16_t magicWord = 0xAF0C;

// canbus things
extern volatile uint8_t TxData[8];
extern volatile uint8_t RxData[8];

uint8_t configureCAN(void);

// Encoder E1 = Encoder(PB4, PB5, 8192*2);
// // interrupt routine intialisation
// void doA(){E1.handleA();}
// void doB(){E1.handleB();}

void setup() {
  pinMode(LED_FAULT, OUTPUT);
  Serial3.begin(115200);
  SimpleFOCDebug::enable(&Serial3);

	uint8_t ret;
	ret = configureCAN();
	if (!ret){
		SIMPLEFOC_DEBUG("CAN init failed.");
		digitalWrite(LED_FAULT, HIGH);
	}

  	if (boardData.canID == 0x000) // If the can ID is not set, then we'll look for a new, free ID.
	{
		uint8_t foundID = FDCAN_FindUniqueID();
		if (foundID != 0)
		{
			boardData.canID = foundID;
			updateData = 1;
			SIMPLEFOC_DEBUG("Unique CAN ID found: %i", foundID);
		} else {
			digitalWrite(LED_FAULT, HIGH);
			SIMPLEFOC_DEBUG("Failed to find a unique CAN ID!");
		}
	}

  //User button setup
  pinMode(BUTTON_PIN, INPUT);
  // Initialize button state to current reading
  last_button_state = digitalRead(BUTTON_PIN);
  button_was_pressed = last_button_state;

  E1.init();
  // E1.enableInterrupts(doA, doB);
  Serial3.println("Encoder ready");
  delay(1000);
  // setting the limits
  M1.velocity_limit = 99999;//99999
  M1.voltage_limit = 1.0;// I=1/0.6 = 1.6A
  M1.current_limit = 2.0;
  DR1.pwm_frequency = 20000;
  DR1.voltage_power_supply = 8.0;
  M1.voltage_sensor_align = 1.0;

  // foc current control parameters
  M1.PID_current_q.P = q_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_q.I= M1.PID_current_q.P*RPHASE/q_phase_inductance;
  M1.PID_current_d.P= d_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_d.I = M1.PID_current_d.P*RPHASE/d_phase_inductance;
  M1.LPF_current_q.Tf = 1/(Tf_ratio*current_bandwidth); 
  M1.LPF_current_d.Tf = 1/(Tf_ratio*current_bandwidth);
  M1.motion_downsample = 0; // - times (default 0 - disabled)

  // velocity PID controller parameters
  M1.PID_velocity.P = M1.PID_current_q.P * vel_bandwidth_ratio; //discrete
  M1.PID_velocity.I = M1.PID_velocity.P*vel_PI_ratio;
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0;
  M1.LPF_velocity.Tf = (1/(Tf_ratio*M1.PID_velocity.P*sample_frequency)); // 5 times the velocity bandwidth
   
  // angle PID controller 
  M1.P_angle.P = M1.PID_velocity.P*pos_bandwidth_ratio*sample_frequency; //continuous
  M1.P_angle.I = (M1.P_angle.P*pos_PI_ratio)/sample_frequency; //discrete
  M1.P_angle.D = 0;
  M1.P_angle.output_ramp = 0;
  M1.LPF_angle.Tf = 0;

  // setting the current sense parameters
  CS1.gain_a *= -1; // invert phase A current sense

  //INIT
  M1.linkSensor(&E1);
  DR1.init();
  M1.linkDriver(&DR1);
  CS1.linkDriver(&DR1);
  CS1.init();
  M1.linkCurrentSense(&CS1);
  //FOC
  M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M1.controller = MotionControlType::angle;
  M1.torque_controller = TorqueControlType::foc_current;
  //START
  M1.init();
  //CS1.skip_align = true;
  M1.initFOC(); //skip for open loop
  Serial.println("***M1 Init***");
  delay(1000);
}

void loop() {
  // --- Button toggle logic ---
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != last_button_state) {
    last_debounce_time = millis();
  }

  if ((millis() - last_debounce_time) > debounce_delay) {
    if (reading != button_was_pressed) {
      button_was_pressed = reading;
      if (button_was_pressed) { // Button just pressed (LOW->HIGH)
        motor_enabled = !motor_enabled;
        if (motor_enabled) {
          M1.enable();
          Serial3.println("Motor ENABLED");
        } else {
          M1.disable();
          Serial3.println("Motor DISABLED");
        }
      }
    }
  }
  last_button_state = reading;

  
  if (loopcounter == loopiter){
    start = micros();
  }

  //MAIN LOOP
  M1.loopFOC();
  M1.move(received_angle);

  if (loopcounter == loopiter){
  //Loop time finish 
  finish = micros();
  looptime = (finish - start);
  current = CS1.getPhaseCurrents();
  foc_currents = CS1.getFOCCurrents(M1.electrical_angle);
  current_magnitude = CS1.getDCCurrent();
  loopcounter = 0;
  }
  loopcounter++;
  // display the angle and the angular velocity to the terminal
  // E1.update();
  // Serial3.print(E1.getAngle());
  // Serial3.print("\t");
  // Serial3.println(E1.getVelocity());
  // Serial3.print("\t");
  // Serial3.println(E1.getStatus());
  // Serial3.print("\t");
  // Serial3.println(E1.getABZResolution());
  // delay(1000);
}

uint8_t configureCAN(void)
{
	FDCAN_Start(0x7CC);
	return 1;
}