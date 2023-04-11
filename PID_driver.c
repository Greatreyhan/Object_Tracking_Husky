#include "PID_driver.h"

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0;
	pid->prevError  = 0.0;

	pid->differentiator  = 0.0;
	pid->prevMeasurement = 0.0;

	pid->out = 0.0;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	// Error signal
    float error = setpoint - measurement;
	
	// Proportional
    float proportional = pid->Kp * error;
	
	// Integral
    pid->integrator = pid->integrator + 0.5 * pid->Ki * pid->T_sample * (error + pid->prevError);

	// Anti-wind-up via integrator clamping 
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }

	// Derivative (band-limited differentiator)
    pid->differentiator = -(2.0 * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0 * pid->tau - pid->T_sample) * pid->differentiator)
                        / (2.0 * pid->tau + pid->T_sample);


	// Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	// Store error and measurement for later use 
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	// Return controller output 
    return pid->out;
}
