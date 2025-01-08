/*  
  AGV BASIC KALMAN LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 14 Desember 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "BasicKalman.h"

Kalman::Kalman(float input_Q, float input_R, float input_P, float input_K){
  Kalman::Q = input_Q;
  Kalman::R = input_R;
  Kalman::P_init = input_P;
  Kalman::K_init = input_K;
}

float Kalman::get_kalman_val(float input_data){
  Kalman::K_init = Kalman::P_init / (Kalman::P_init + Kalman::R);
  Kalman::Est_val = Kalman::Est_val + Kalman::K_init * (input_data - Kalman::Est_val);
  Kalman::P_init = (1 - Kalman::K_init) * Kalman::P_init + Kalman::Q;

  return Kalman::Est_val;
}