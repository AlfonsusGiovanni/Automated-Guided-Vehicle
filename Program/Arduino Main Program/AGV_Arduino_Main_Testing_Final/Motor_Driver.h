/*  
  AGV MOTOR DRIVER LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 11 NOVEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "Arduino.h"

enum active_type {ACTIVE_LOW, ACTIVE_HIGH};
enum motor_dir {CW, CCW};

class Motor_Driver{
  private:
    uint8_t
    ena_Pin, pwm_Pin, dir_Pin, sig_Pin;

    uint8_t
    active_mode,
    base_speed,
    max_speed;

  public:
    uint8_t 
    L_speed,
    R_speed;

    Motor_Driver(const uint8_t input_basespeed, const uint8_t input_maxspeed, active_type mode);

    void driver_Pinset(const uint8_t enaPin, const uint8_t pwmPin, const uint8_t dirPin, const uint8_t sigPin);

    void driver_Enable(void);
    void driver_Disable(void);

    void motor_Run(void);
    void motor_Stop(void);

    void set_Dir(motor_dir input_dir);
    void set_Speed(uint8_t input_speed);
};

#endif