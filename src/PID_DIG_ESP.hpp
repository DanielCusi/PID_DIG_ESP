/*
  PID_DIG_ESP, a digital PID with timer implementation for ESP32
  Copyright (C) 2024  Daniel Cusí

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef PID_DIS_ESP
#define PID_DIG_ESP

// Check if using arduino
#ifdef ARDUINO
#include <Arduino.h>  // Add the library
#endif

#include "esp_err.h"    // Error codes, esp_err_t

#include <stdbool.h>    // For saturation enable boolean

// Structures and typedefs, to simplify library use
// PID class
typedef struct PID_params {
  // Parameters
  float kp;   // Proportional, use for "current" error. Corrects the current error between the setpoint (input) and the output.
  float ki;   // Integral, use for "past" error. Corrects the integral error, e.g. distance if it's controlling speed. Corrects error at infinity, but too high may overshoot and oscillate. Slow acting.
  float kd;   // Derivative, use for "future" error. Acts upon the error derivate, so anticipates the change. It acts fast, dampens the oscillations and overshoot, but sensitive to noise.

  // Saturation, for integral and derivative use negatives to not use them, and it clamps in both negative and positive.
  float integral_acc_saturation;  // Integral accumulator saturation. Avoids carrying too muche error over time, e.g. a motor that has been blocked, 
                                  // if saturation isn't used, will run for long to compensate the error over time. Negative to ignore, limits in positive and negative.
  float derivative_saturation;    // Derivative saturation. If there is a huge change at some point, it might cause too much effect in the output, if not used.
                                  // Negative to ignore, limits in positives and negatives.
  bool enable_output_saturation;  // Enables output saturation, as both upper and lower limit can be negatives.
  float output_saturation_upper;  // Output saturation, upper limit. Used for physical limits of the actuator, e.g. if it's a percent, its limit is 100. Limits the high value.
  float output_saturation_lower;  // Output saturation, lower limit. Used for physical limits of the actuator, e.g. if it can't go to negatives, is 0. Limits the low value.
} PID_params;

/**
 * @brief PID class, creates an object of PID class calling PID <name>(<PID_params>).
 *        It is a digital PID that will execute its cycle each time the cycle function is called, which can be manually handled, preferably with a timer, or
 *        it can be used with the PID_timer class included in the library, which handles timer itself or manually and allows assigning functions for the setpoint and output handling.
 *        Just keep in mind that a constant cycle time is important, so a timer is recommended.
*/
class PID{
  private:
    // Internal use
    double integral_accumulator = 0;  // Stores the integral error accumulator, the error through time
    float previous_error = 0;         // Stores the previous error, used for calculating the derivative
    PID_params parameters = {0};      // Stores the PID's parameters, consult the structure.

  public:
    float output = 0;                 // PID output, use (read) for acting upon the system

    /**
     * @brief PID constructor
     * 
     * @param PID_parameters Parameters to define the PID.
    */
    PID(PID_params PID_parameters = {.kp = 0, .ki = 0, .kd = 0, .integral_acc_saturation = -1, .derivative_saturation = -1,
                                     .enable_output_saturation = false, .output_saturation_upper = -1, .output_saturation_lower = -1});

    /**
     * @brief Run a PID cycle
     * 
     * @param input Input value, the mesured value of the output
     * @param setpoint Target value of the output
     * 
     * @returns PID output value, to use as an input for the controlled system
     */
    float cycle(float input, float setpoint);

    // Return a copy of the parameters, as it is a copy they are read only.
    PID_params get_parameters();

    // Set new PID parameters
    void set_parameters(PID_params PID_parameters);
};

class PID_timer : public PID{
  private:
    static uint next_timer_number;                                      // Timer number to use in the next PID timer, for automatically setting it. Static
    static PID_timer ** timer_object_callback[8];                       // An array of an array of PID_timer pointers. To use in the timer interrupt, so an array of cycles is called for syncronous PIDs.
    #ifdef Arduino_h
    // Using arduino timer's implementation
    hw_timer_t* timer = NULL;                                           // Arduino's timer variable
    #else
    // esp-idf is being used, use timer handle
    #warning "PID_DIG_ESP is not implemented for esp-idf for now"
    #endif
    void (*pre_cycle_function)(float* input, float* setpoint) = NULL;   // Pointer to the pre-cycle function, to get the input and the setpoint
    void (*post_cycle_function)(float output) = NULL;                   // Pointer to the post-cycle function, to handle the output as desired (act on actuator)
    //std::function<void(float *, float *)> pre_cycle_method = NULL;      // Pointer to method, to allow assigning a method to set the input and setpoint
    //std::function<void(float)> post_cycle_method = NULL;                // Pointer to method, to allow assigning a method to top handle the output
    uint timer_number = 0;                                              // Timer number of this PID
    uint32_t timer_period_us = 0;                                       // Timer period in us, the maximum is 1 s
  public:
    float input;                                                        // PID's input, the reading of the process output. Can be set manually, especially if not using functions
    float setpoint;                                                     // PID's setpoint, the output's target value. Can be set manually, especially if not using functions

    /**
     * @brief PID with timer constructor. It doesn't initialize nor begin the timer, call initialize_timer to do it.
     * 
     * @param PID_parameters Parameters to define the PID.
     * @param period PID's cycle period, in us. Recommended 10 ms, higher than 100 ms can be too much for some systems. If manual
     *               control is desired, set whatever value and don't call initialize_timer, instead call timer_cycle manually.
     * @param timer_num Number of timer to use, set to -1 to use the last timer used + 1, beginning with 0. Can be used for the first
     *                  one only to reserve timers for other uses. As for now, each PID needs its own timer, it should change in the future.
     * @param pre_cycle Function to run before the PID's cycle is executed, which receives the input pointer as the first parameter and
     *                  the setpoint as the second parameter, to which it should assign the desired value. Set to NULL to not use it.
     * @param post_cycle Function to run after the PID's cycle is executed, which receives the PID's output value, and can
     *                   use it to act on the system. Set to NULL to not use it.
    */
    PID_timer(PID_params PID_parameters = {.kp = 0, .ki = 0, .kd = 0, .integral_acc_saturation = -1, .derivative_saturation = -1,
                                           .enable_output_saturation = false, .output_saturation_upper = -1, .output_saturation_lower = -1},
              uint32_t period = 0, int timer_num = -1, void (*pre_cycle)(float *, float *) = NULL, void (*post_cycle)(float) = NULL);

    /**
     * @brief Initializes the timer, assigns the interruption and starts it.
     * 
     * @returns returns ESP_OK (0) if it has been correctly initialized and started.
     *          returns ESP_ERR_INVALID_ARG if the period can't be set, probably too high or 0.
     *          returns ESP_ERR_INVALID_STATE if the timer is already running.
     *          returns ESP_FAIL if it couldn't be started, maybe the timer number doesn't exist.
    */
    virtual esp_err_t initialize_timer();

    // Timer cycle, first runs the pre_cycle function if it's not NULL, then runs the PID, and finally runs post_cycle if it's not NULL
    // Can be used in an interruption
    void IRAM_ATTR timer_cycle();

    // Methods to be overriden for cycle use
    virtual void pre_cycle_method(float * input, float * setpoint){};
    virtual void post_cycle_method(float output){};

    // Returns the timer number used in the object
    uint get_timer_number();

    // For PID cycle time testing
    #ifdef PID_ESP_DIG_DEBUG_CYCLE_TIME
      unsigned long cycle_time_test;
    #endif
};

#endif