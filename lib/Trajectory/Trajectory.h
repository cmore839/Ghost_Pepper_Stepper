#pragma once

#include <SimpleFOC.h>

// A simple trapezoidal trajectory planner
class Trajectory {
public:
    void plan(float current_pos, float target_pos, float max_vel, float max_accel) {
        p0 = current_pos;
        p1 = target_pos;
        v_max = abs(max_vel);
        a_max = abs(max_accel);

        float delta_p = p1 - p0;
        direction = sign(delta_p);
        
        // This is a simplified trapezoidal profile. A full S-curve could be
        // implemented here as a next step by adding a jerk parameter.
        t_ramp = v_max / a_max;
        p_ramp = 0.5f * a_max * t_ramp * t_ramp;

        if (abs(delta_p) < 2 * p_ramp) { // Triangular move (doesn't reach max_vel)
            t_ramp = sqrt(abs(delta_p) / a_max);
            t_total = 2 * t_ramp;
            v_actual_max = a_max * t_ramp;
            t_cruise = 0;
        } else { // Trapezoidal move
            p_cruise = abs(delta_p) - 2 * p_ramp;
            t_cruise = p_cruise / v_max;
            t_total = 2 * t_ramp + t_cruise;
            v_actual_max = v_max;
        }
        
        is_planned = true;
        start_time = _micros();
    }

    float getPosition(unsigned long now) {
        if (!is_planned) return p1; // If not planned, hold the final position

        float t = (now - start_time) / 1000000.0f; // time in seconds

        if (t >= t_total) {
            is_planned = false; // Move is complete
            return p1;
        }

        float pos;
        if (t < t_ramp) {
            pos = p0 + direction * 0.5f * a_max * t * t;
        } else if (t < t_ramp + t_cruise) {
            float p_so_far = direction * 0.5f * a_max * t_ramp * t_ramp;
            pos = p0 + p_so_far + direction * v_actual_max * (t - t_ramp);
        } else {
            float t_decel = t - (t_ramp + t_cruise);
            float p_so_far = direction * (p_ramp + v_actual_max * t_cruise);
            // The deceleration phase requires careful calculation from the end of the cruise phase
            pos = p0 + p_so_far + (direction * v_actual_max * t_decel) - (direction * 0.5f * a_max * t_decel * t_decel);
        }
        return pos;
    }

    bool isDone() {
        return !is_planned;
    }

private:
    float p0, p1, v_max, a_max, v_actual_max;
    float t_ramp, t_cruise, t_total, p_ramp, p_cruise; // <-- FIX: Added missing declarations
    int direction;
    bool is_planned = false;
    unsigned long start_time;

    int sign(float val) {
        return (0.0f < val) - (val < 0.0f);
    }
};