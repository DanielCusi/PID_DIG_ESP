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

// System to control for testing purposes, a motor.
// Current values the motor, for calculation. We suppose it starts stopped.
float current_current = 0;
float current_speed = 0;
float current_speed_rpm = 0;    // Internally the speed is in rad/s, so use this variable to get the rpm.
void controlled_system(float input_voltage){   // A system to which the PID acts, as an example
    // A DC motor system, the input is the voltage
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

    // Transform the speed to rpm, as in the formulas it is in radians/s
    current_speed_rpm = current_speed / (2 * PI) * 60;
}

// Function that sets the PID input and the setpoint
void set_PID_input_setpoint(float * input, float * setpoint){
    // As the arguments are pointers, they have to be dereferenced to be changed
    *input = current_speed_rpm;
    // Setpoint can be kept as is
}

/*
    Create the PID timer object with the parameters, with a 10 ms period, and set the process function to run after the PID cycle
    The second argument is the cycle period, in us. The third is the timer number to use (-1 to use last + 1).
    The fourth is the pre cycle function. This function will be called before running the PID cycle, don't print to serial nor use delays.
    It must take two float pointer arguments(* float), the first is the value to control (e.g. speed of motor), and the second is the setpoint (the desired value of the value).
    Set the values by dereferencing the arguments, that is, if the first argument is float * input, set the value with *input = 10.
    The fifth is the post cycle function. It takes only a float argument, the PID output, and can be used to act on the process (e.g., the motor driver)
    If no function is desired to run, set the argument to NULL.
*/
PID_timer PID_example(parameters, 10 * 1000, -1, set_PID_input_setpoint, controlled_system);

void setup() {
    Serial.begin(115200);   // On ESP32 it's not required
    printf("cycle(x10 ms), voltage(V), speed(rpm)\n");  // We'll print as a CSV

    // Set the PID setpoint, the value that it will try to have the controlled system take.
    PID_example.setpoint = 200.0;    // Target speed of the motor, in rpm

    // Start the PID timer
    PID_example.initialize_timer();

    // We'll run the PID just for a second
    for(int i = 0; i < 1 * 1000 / 10; i++){
        // The PID will run on the background, so we don't have to do anything
        // To control the PID input, which is the controlled system output, it can be done with the function pointer or manually updating
        // input attribute. As the input must be updated each cycle, the most reliable way is using the function. The setpoint can be updated
        // at any moment.

        // Print the data
        printf("%i, %f, %f\n", i, PID_example.output, current_speed_rpm);
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