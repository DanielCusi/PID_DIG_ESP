# Digital PID for ESP32

Digital PID for ESP32, which can be used manually or with a timer handled by the library. The PID expects a constant cycle time, take it into account if not using the timer.

I did this library because the libraries I found didn't have a timer integrated with the PID, and I wanted to make it as easy as possible to use.

I'm going to submit another library to drive a motor soon.

## To do

This library uses the old timer API of the esp32 arduino core, version 2. With the new 3 version, the timer initialization doesn't require timer number, as it is handled internally. As I'm using platformio I'm using version 2, and I haven't had time to implement it in version 3. Version 3 will be implemented in the future.

I also want to run all the PID at the same time in the future, using the same timer, to save on timers and to avoid delays between PIDs, as when controlling two motors on a drivetrain.

## How to use

In the examples folder I have written two examples, without and with timer. Refer to those for more information, and check the PID_ESP_DIG.hpp for more detailed information.

### Basic PID

The basic PID takes the PID_params structure, and has to be run by calling the cycle method.

PID_params structure
```
typedef struct PID_params {
  // Parameters
  float kp;   // Proportional
  float ki;   // Integral
  float kd;   // Derivative

  // Saturation, for integral and derivative use negatives to not use them, and it clamps in both negative and positive.
  float integral_acc_saturation;  // Integral accumulator saturation.
  float derivative_saturation;    // Derivative saturation.

  bool enable_output_saturation;  // Enables output saturation
  float output_saturation_upper;  // Output saturation, upper limit.
  float output_saturation_lower;  // Output saturation, lower limit.
} PID_params;
```
This structure is what sets the PID parameters. The kp, ki and kd are the proportional, integral and derivative components of the PID. The saturation for the integral and derivative allows to limit their outputs, for integral can avoid storing too much error, for example if a motor gets stuck. Derivative may be used to avoid effects of sudden changes. And the output saturation can be used to limit the output to the actuator input limits, for example a 0-1023 10 bit PWM, or -12-12 V controller.

The PID object is created as follows:`PID your_PID_name(parameters);`. With parameters being a PID_params structure.

For running a cycle, call `float cycle(float input, float setpoint);` method, as `float ouput = your_PID_name.cycle(input, setpoint)`. The input is the current value of what is being controlled, as the speed of a motor, the setpoint is the value that is desired, and the return value is the output to pass to the actuator. It has to be called with a constant period, if not it will become unstable.

### Timer PID

The library also has a PID class which includes a timer. The advantage is that it will run on the background, and at a constant frequency. As for now, each PID uses a timer, which means that the number of PIDs are limited to the ESP32 timers, which can be 2 or 4 depending on the board.

PID_timer object creation
```
PID_timer(PID_params PID_parameters, uint32_t period, int timer_num, void (*pre_cycle)(float *, float *), void (*post_cycle)(float));
```
The timer PID also uses the parameters structure, but it also needs the period in us (take into account that it can't go faster than the cycle time to run), the timer number (-1 to use last used+1, starting at zero), and allows for two functions to be called, a function to be run before the PID cycle, and another before. These are called from an interrupt, so don't print to serial, use delays or have long execution time. If they are unused, pass NULL, but they are recommended, as the PID input should be updated each time just before it runs, and the output should act upon the system as soon as the PID runs.

The first function allows setting the input and setpoint of the PID. It should read the value of what is being controlled at that moment. Although the input has to be updated, the setpoint can be ignored, and instead be updated manually with the object attribute PID_timer.setpoint.

Example of input function:
```
void set_PID_input_setpoint(float * input, float * setpoint){
    // As the arguments are pointers, they have to be dereferenced to be changed
    *input = current_speed_rpm;
    // Setpoint can be kept as is
}
```
Instead of the variable, it can read a pulse counter from an encoder, for example.

The second function receives the PID output, and can use it to act upon the system.

Example of output function
```
void act_on_system(float PID_output){
    ledcWrite(21, PID_output * MAX_DUTY);
}
```
In this case, the PID should be limited from 0 to 1, or instead remove MAX_DUTY and limit it from 0 to MAX_DUTY.

It also includes two methods, pre_cycle_method and post_cycle_method, that can be used for child classes, and act hte same as the function pointers.

And for last, it allows checking the time to run the cycle (including pre and post functions) using cycle_time_test attribute, which is in us. To use it, add `#define PID_ESP_DIG_DEBUG_CYCLE_TIME` before including the library.

## Dependencies

This library requires BindArg: https://github.com/openlab-vn-ua/BindArg