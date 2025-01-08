/*  
  AGV BASIC KALMAN LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 14 Desember 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#ifndef BASIC_KALMAN_H
#define BASIC_KALMAN_H

class Kalman{
  private:
    float
    Q = 0.1,
    R = 0.5,
    P_init,
    K_init;
  public:
    float 
    Est_val;

    Kalman(float input_Q, float input_R, float input_P, float input_K);
    float get_kalman_val(float input_data);
};

#endif