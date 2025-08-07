// motor_characterization.cpp
#include "motor_characterization.h"

// Private helper function to measure resistance using Phase B
float measureResistance(FOCMotor& motor, BLDCDriver& driver, float voltage) {
    // The current sense is already calibrated in the main setup() function.
    driver.enable();
    driver.setPwm(0, voltage, 0); // Apply voltage to Phase B
    SIMPLEFOC_DEBUG("MOT: Test voltage %.2fV applied to Phase B.", voltage);
    _delay(1000); // Wait for the current to settle

    PhaseCurrent_s current = motor.current_sense->getPhaseCurrents();

    driver.setPwm(0, 0, 0);
    _delay(200);
    driver.disable();
    SIMPLEFOC_DEBUG("MOT: Test voltage removed.");
    
    SIMPLEFOC_DEBUG("MOT-DEBUG: Calculated Current (B): %.4f A", current.b);
    
    float R = 0;
    // Check if measured current is sensible
    if (abs(current.b) > 0.01) {
        R = voltage / current.b;
    } else {
        SIMPLEFOC_DEBUG("MOT: Characterization failed. No current measured on Phase B.");
        R = 0;
    }

    return R;
}


// Main characterization function
void characteriseMotor(FOCMotor& motor, BLDCDriver& driver, float voltage) {
    SIMPLEFOC_DEBUG("MOT: Starting motor characterization.");

    // 1. Measure Phase Resistance on Phase B
    SIMPLEFOC_DEBUG("MOT: Measuring phase resistance.");
    motor.phase_resistance = measureResistance(motor, driver, voltage);

    if (motor.phase_resistance <= 0) {
        SIMPLEFOC_DEBUG("MOT: ERROR: Measured resistance is 0 or negative. Aborting.");
        return;
    }
    SIMPLEFOC_DEBUG("MOT: Measured R: %.4f Ohms", motor.phase_resistance);

    // 2. Measure Phase Inductance using the time constant method on Phase B
    SIMPLEFOC_DEBUG("MOT: Measuring phase inductance.");

    long t_start = _micros();
    driver.enable();
    driver.setPwm(0, voltage, 0); // Apply voltage to Phase B
    
    // Wait for current to reach 63.2% of its max value (V/R)
    while(_micros() - t_start < 1000000) { // 1 second timeout
        PhaseCurrent_s current = motor.current_sense->getPhaseCurrents();
        if (abs(current.b) > 0.632f * (voltage / motor.phase_resistance)) {
            break;
        }
    }
    long t_end = _micros();

    driver.setPwm(0, 0, 0);
    driver.disable();

    float time_constant = (t_end - t_start) * 1e-6f; // Time in seconds
    motor.phase_inductance = motor.phase_resistance * time_constant;

    // *** CORRECTED PRINT STATEMENT FOR INDUCTANCE ***
    // Use standard Serial.print to bypass the limited SIMPLEFOC_DEBUG macro
    Serial3.print(F("MOT: Measured L: "));
    Serial3.print(motor.phase_inductance, 6); // Print float with 6 decimal places
    Serial3.println(F(" H"));

    SIMPLEFOC_DEBUG("MOT: Characterization complete.");
}