/*  
  AGV MOTOR DRIVER LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 11 November 2024uint8_t
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "Motor_Driver.h"

Motor_Driver::Motor_Driver(const uint8_t input_basespeed, const uint8_t input_maxspeed, active_type mode){
  Motor_Driver::active_mode = mode;
  Motor_Driver::base_speed = input_basespeed;
  Motor_Driver::max_speed = input_maxspeed;
}

void Motor_Driver::driver_Pinset(const uint8_t enaPin, const uint8_t pwmPin, const uint8_t dirPin, const uint8_t sigPin){
  Motor_Driver::ena_Pin = enaPin;
  Motor_Driver::pwm_Pin = pwmPin;
  Motor_Driver::dir_Pin = dirPin;
  Motor_Driver::sig_Pin = sigPin;
  
  pinMode(enaPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(sigPin, OUTPUT);
}

void Motor_Driver::motor_Run(){
  if(Motor_Driver::active_mode == ACTIVE_LOW){
    digitalWrite(Motor_Driver::ena_Pin, LOW);
  }

  else if(Motor_Driver::active_mode == ACTIVE_HIGH){
    digitalWrite(Motor_Driver::ena_Pin, HIGH);
  }
}

void Motor_Driver::motor_Brake(){
  if(Motor_Driver::active_mode == ACTIVE_LOW){
    digitalWrite(Motor_Driver::ena_Pin, HIGH);
  }

  else if(Motor_Driver::active_mode == ACTIVE_HIGH){
    digitalWrite(Motor_Driver::ena_Pin, LOW);
  }
}

void Motor_Driver::motor_Start(){
  if(Motor_Driver::active_mode == ACTIVE_LOW){
    digitalWrite(Motor_Driver::sig_Pin, LOW);
  }

  else if(Motor_Driver::active_mode == ACTIVE_HIGH){
    digitalWrite(Motor_Driver::sig_Pin, HIGH);
  }
}

void Motor_Driver::motor_Stop(){
  if(Motor_Driver::active_mode == ACTIVE_LOW){
    digitalWrite(Motor_Driver::sig_Pin, HIGH);
  }

  else if(Motor_Driver::active_mode == ACTIVE_HIGH){
    digitalWrite(Motor_Driver::sig_Pin, LOW);
  }
}

void Motor_Driver::set_Dir(motor_dir input_dir){
  if(Motor_Driver::active_mode == ACTIVE_LOW){
    if(input_dir == CW){
      digitalWrite(Motor_Driver::dir_Pin, LOW);
    }
    
    else if(input_dir == CCW){
      digitalWrite(Motor_Driver::dir_Pin, HIGH);
    }
  }

  else if(Motor_Driver::active_mode == ACTIVE_HIGH){
    if(input_dir == CW){
      digitalWrite(Motor_Driver::dir_Pin, HIGH);
    }
    
    else if(input_dir == CCW){
      digitalWrite(Motor_Driver::dir_Pin, LOW);
    }
  }
}

void Motor_Driver::set_Speed(int input_speed){
  int speed = input_speed;

  if(speed < 0) speed = 0;
  else if(speed >= Motor_Driver::max_speed) speed = Motor_Driver::max_speed;
  analogWrite(Motor_Driver::pwm_Pin, input_speed);
}
