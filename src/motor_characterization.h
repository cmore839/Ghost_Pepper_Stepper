// motor_characterization.h
#ifndef MOTOR_CHARACTERIZATION_H
#define MOTOR_CHARACTERIZATION_H

#include <SimpleFOC.h>

/**
 * @brief Standalone motor characterization function to measure phase resistance and inductance.
 *
 * @param motor The FOCMotor object to be characterized.
 * @param driver The BLDCDriver object linked to the motor.
 * @param voltage The voltage to use for the characterization test.
 */
void characteriseMotor(FOCMotor& motor, BLDCDriver& driver, float voltage);

#endif