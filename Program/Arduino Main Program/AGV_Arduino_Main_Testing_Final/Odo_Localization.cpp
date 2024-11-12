/*  
  AGV ODOMETRY LOCALIZATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 11 NOVEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "Odo_Localization.h"

static uint8_t
  wl_pulseA, wl_pulseB,
  wl_pulseC, prev_wl_pulseC,
  wr_pulseA, wr_pulseB,
  wr_pulseC, prev_wr_pulseC;

static unsigned long
  wl_timer, prev_wl_timer,
  wr_timer, prev_wr_timer;

static double 
  left_distance, prev_left_distance,
  right_distance, prev_right_distance;

Odometry::Odometry(const double &input_diameter, const double &input_length, const uint8_t input_encRes){
  Odometry::wD = input_diameter;
  Odometry::wL = input_length;

  Odometry::wC = 2 * M_PI * (input_diameter / 2);
  Odometry::wRes = Odometry::wC / input_encRes;
}

void Odometry::enc_Pinset(const uint8_t input_LpinA, const uint8_t input_LpinB, const uint8_t input_LpinC, const uint8_t input_RpinA, const uint8_t input_RpinB, const uint8_t input_RpinC){
  Odometry::encL_pinA = input_LpinA;
  Odometry::encL_pinB = input_LpinB;
  Odometry::encL_pinC = input_LpinC;

  Odometry::encR_pinA = input_RpinA;
  Odometry::encR_pinB = input_RpinB;
  Odometry::encR_pinC = input_RpinC;
  
  pinMode(input_LpinA, INPUT);
  pinMode(input_LpinB, INPUT);
  pinMode(input_LpinC, INPUT);

  pinMode(input_RpinA, INPUT);
  pinMode(input_RpinB, INPUT);
  pinMode(input_RpinC, INPUT);
}

void Odometry::enc_Read(){
  //wl_pulseA = digitalRead(Odometry::encL_pinA);
  //wr_pulseA = digitalRead(Odometry::encR_pinA);

  //wl_pulseB = digitalRead(Odometry::encL_pinB);
  //wr_pulseB = digitalRead(Odometry::encR_pinB);

  wl_pulseC = digitalRead(Odometry::encL_pinC);
  wr_pulseC = digitalRead(Odometry::encR_pinC);

  if(prev_wl_pulseC == HIGH)
    if(wl_pulseC == LOW)
      Odometry::wl_counter+=1;

  if(prev_wr_pulseC == HIGH)
    if(wr_pulseC == LOW)
      Odometry::wr_counter+=1;

  prev_wl_pulseC = wl_pulseC;
  prev_wr_pulseC = wr_pulseC;
}

void Odometry::enc_Reset(){
  Odometry::wl_counter = 0;
  Odometry::wr_counter = 0;
}

void Odometry::est_Speed(){
  double
  delta_left_distance,
  delta_right_distance;

  left_distance = Odometry::wl_counter * Odometry::wRes,
  right_distance = Odometry::wr_counter * Odometry::wRes;

  if(millis() - prev_wl_timer > 100){
    delta_left_distance = left_distance - prev_left_distance;
    Odometry::lspeed = delta_left_distance * 10;

    prev_left_distance = left_distance;  
    prev_wl_timer = millis();
  }

  if(millis() - prev_wr_timer > 100){
    delta_right_distance = right_distance - prev_right_distance;
    Odometry::rspeed = delta_right_distance * 10;

    prev_right_distance = right_distance;
    prev_wr_timer = millis();
  }
}

void Odometry::get_Pos(){
  Odometry::y = (left_distance + right_distance) / 2;
}

void Odometry::reset_Pos(){
  Odometry::x = 0;
  Odometry::y = 0;
  Odometry::theta = 0;
}

void Odometry::goTogoal(const double &inputX, const double &inputY, const double &inputTheta){
  if(Odometry::x != inputX && Odometry::y != inputY && Odometry::theta != inputTheta){
    get_Pos();

    if(inputX == 0 && inputY != 0){
      
    }
  }
}
