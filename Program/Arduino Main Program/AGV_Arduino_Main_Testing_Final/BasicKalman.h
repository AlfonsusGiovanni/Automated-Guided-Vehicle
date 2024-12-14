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
    P_init = 1.0,
    K_init = 0.5;
  public:
    float 
    Est_val,  // Nilai estimasi
    P_val,    // Estimasi nilai variasi error
    K_val;    // 

    float get_kalman_val(float input_data);
};

#endif