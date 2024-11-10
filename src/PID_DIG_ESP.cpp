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

#include "PID_DIG_ESP.hpp"

#include "BindArg.h"  // Library that allows assigning object functions to interrupts

#include <math.h>     // For signbit and lrint

#include "esp_log.h"  // For test
#ifdef Arduino_h
// Arduino definitions, used if arduino is included, in platformio include the library after Arduino.h
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #error "Not implemented for arduino version  >= 3"
#endif

// Static initializations
uint PID_timer::next_timer_number = 0; // Timer number to assign next PID defined

// PID functions
// Constructor
PID::PID(PID_params PID_parameters){
  parameters.kp = PID_parameters.kp;
  parameters.ki = PID_parameters.ki;
  parameters.kd = PID_parameters.kd;

  parameters.integral_acc_saturation = PID_parameters.integral_acc_saturation;
  parameters.derivative_saturation = PID_parameters.derivative_saturation;

  parameters.enable_output_saturation = PID_parameters.enable_output_saturation;
  parameters.output_saturation_upper = PID_parameters.output_saturation_upper;
  parameters.output_saturation_lower = PID_parameters.output_saturation_lower;
}

float PID::cycle(float input, float setpoint){
  float sum = 0;
  float error = setpoint-input;

  // Proportional
  sum = error * parameters.kp;

  // Integral
  integral_accumulator += error;  // No overflow test as it is float
  // Integral saturation
  if(parameters.integral_acc_saturation > 0){
    if(integral_accumulator > parameters.integral_acc_saturation){
      integral_accumulator = parameters.integral_acc_saturation;
    }else if(integral_accumulator < (-parameters.integral_acc_saturation)){
      integral_accumulator = -parameters.integral_acc_saturation;
    }
  }
  float integral_output = integral_accumulator * parameters.ki;
  sum += integral_output;


  // Derivate
  float derivate_output = (error - previous_error) * parameters.kd;
  previous_error = error;
  if(parameters.derivative_saturation > 0){
    if(derivate_output > parameters.derivative_saturation){
      derivate_output = parameters.derivative_saturation;
    }else if(derivate_output < (-parameters.derivative_saturation)){
      derivate_output = -parameters.derivative_saturation;
    }
  }
  sum += derivate_output;

  // Output, process its saturation
  if(parameters.enable_output_saturation){
    if(sum > parameters.output_saturation_upper){
      output = parameters.output_saturation_upper;
    }else if(sum < parameters.output_saturation_lower){
      output = parameters.output_saturation_lower;
    }else{
      output = sum;
    }
  }else{
    output = sum;
  }

  return output;
}

// PID timer functions
// Constructor
PID_timer::PID_timer(PID_params PID_parameters, uint32_t period, int timer_num,
                     void (*pre_cycle)(float *, float *), void (*post_cycle)(float)) :
                     PID(PID_parameters){
  
    timer_period_us = period;
    if(timer_num < 0){
      timer_number = next_timer_number++;
    } else{
      timer_number = timer_num;
    }
    pre_cycle_function = pre_cycle;
    post_cycle_function = post_cycle;
}

esp_err_t PID_timer::initialize_timer(){
  if(!timer_period_us || timer_period_us > 1000000) return ESP_ERR_INVALID_ARG;
  #if ESP_ARDUINO_VERSION_MAJOR == 2
  timer = timerBegin(timer_number, 80, true); // Timer at 1 MHz, 1 us resolution
  #else
  #error "Not implemented for arduino version  >= 3"
  #endif
  if(timer == NULL) return ESP_FAIL;
  timerAttachInterrupt(timer, bindArgGateThisAllocate(&PID_timer::timer_cycle, this), true);
  timerAlarmWrite(timer, timer_period_us, true);
  timerAlarmEnable(timer);
  timerStart(timer);
  return ESP_OK;
}

void IRAM_ATTR PID_timer::timer_cycle(){
  #ifdef PID_ESP_DIG_DEBUG_CYCLE_TIME
    cycle_time_test = micros();
  #endif
  // Run the pre cycle if exists, to assign input and setpoint
  if(pre_cycle_function != NULL){
    // Function pointer
    pre_cycle_function(&input, &setpoint);
  } else{
    // Method
    pre_cycle_method(&input, &setpoint);
  }

  // Execute the PID cycle
  cycle(input, setpoint);

  // Run the post cycle to handle the output
  if(post_cycle_function != NULL){
    // Function pointer
    post_cycle_function(output);
  } else{
    // Method
    post_cycle_method(output);
  }
  #ifdef PID_ESP_DIG_DEBUG_CYCLE_TIME
    cycle_time_test = micros()-cycle_time_test;
  #endif
}

uint PID_timer::get_timer_number(){
  return timer_number;
}

#else
//Arduino not declared, definitions for esp-idf
#error "ESP-IDF implementation not done, for now include arduino component"

#endif