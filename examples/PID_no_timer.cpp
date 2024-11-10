#include <Arduino.h>    // Needed if using platformio

#include "PID_DIG_ESP.hpp"

// Define the PID parameters in the structure
// These values change depending on what it is controlling, the PID cycle frequency and the desired response.
// To get good starting values, the Ziegler-Nichols method can be used: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
// In this example, the Ku is 0.048, and the Tu is 8 * 0.01 = 0.08 s
// Although it is unstable in this case, so it has to be tuned by hand. You can try to improve the values.
// Look at https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller to learn more.
PID_params parameters = {
  .kp = 0.02,                           // Proportional
  .ki = 0.003,                          // Integral
  .kd = 0.02,                           // Derivative

  .integral_acc_saturation = -1,        // Integral saturation, used for example if a motor gets stuck, to avoid it spinning for a long time after unstucking it
  .derivative_saturation = -1,          // Derivative saturation, to limit the effects of sudden changes, for example at start

  .enable_output_saturation = true,     // Output saturation, limits PID output, as the actuator (or whatever) may only accept a range of values. For example, a 12 V motor driver.
  .output_saturation_upper = 12.0,      // Upper limit of the saturation, can be negative, but not less than the lower
  .output_saturation_lower = -12.0,     // Bottom limit of the saturation, can be positive, but not more than the upper
};

PID PID_example(parameters);    // Create the PID object with the parameters

// System to control for testing purposes, a motor.
// Current values the motor, for calculation. We suppose it starts stopped.
float current_current = 0;
float current_speed = 0;
float controlled_system(float input_voltage){   // A system to which the PID acts, as an example
    // A DC motor system, the input is the voltage, and the output is the rpm on the next cycle.
    // Motor torque: Tm = Ki*ia
    // E.M.F.: e = Kb*w
    // Acceleration (torque balance): J * dw/dt = Tm - b * w
    // Electric circuit: R * ia + L * di/dt = ea - e

    // Constants
    const float d_time = 0.01;  // Time between reading, to apply the current and speed changes.
    const float Ki = 0.01;      // Nm/A, par/A
    const float Kb = 0.01;      // V/A, Voltage/rpm
    const float J = 1.16E-6;    // kgm^2/s^2, rotational inertia
    const float b = 0.000047;   // Nms, rotational friction
    const float R = 10.6;       // ohm, Motor resistance
    const float L = 0.82;       // henry, motor inductance

    // Calculations
    float Tm = Ki * current_current;                                        // Motor torque
    float fem = Kb * current_speed;                                         // Motor EMF
    float accel = (Tm - b * current_speed) / J;                             // Motor acceleration
    float der_curr = ((input_voltage - fem) - (R * current_current)) / L;   // Current derivative

    // Apply speed and current changes, supposing 10 ms time
    current_current += der_curr * d_time;
    current_speed += accel * d_time;

    // Return the speed in rpm, as in the formulas it is in radians/s
    return current_speed / (2 * PI) * 60;
}

void setup() {
    Serial.begin(115200);   // On ESP32 it's not required
    printf("cycle(x10 ms), voltage(V), speed(rpm)\n");  // We'll print as a CSV

    // We'll run the PID just for a second
    const float setpoint = 200.0;    // Target speed of the motor, in rpm
    float control_voltage = 0;      // Voltage supplied to the motor
    float motor_speed = 0;          // Motor speed, in rpm
    for(int i = 0; i < 1 * 1000 / 10; i++){
        // Run the PID, with 10 ms cycle
        // Output the information on the terminal
        printf("%i, %f, %f\n", i, control_voltage, motor_speed);
        // Get the PID output
        control_voltage = PID_example.cycle(motor_speed, setpoint);
        // Act upon the system
        motor_speed = controlled_system(control_voltage);
        // Delay
        delay(10); 
    }

    // Flush the text, and go to sleep
    fflush(stdout);
    fsync(fileno(stdout));
    esp_deep_sleep_start();
}

void loop() {

}