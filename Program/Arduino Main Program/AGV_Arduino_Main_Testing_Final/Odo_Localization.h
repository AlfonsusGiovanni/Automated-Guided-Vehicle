/*  
  AGV ODOMETRY LOCALIZATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 11 NOVEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#ifndef ODO_LOCALIZATION_H
#define ODO_LOCALIZATION_H

#include "Arduino.h"
#include "math.h"

class Odometry{
private:
    uint8_t
    encL_pinA, encL_pinB, encL_pinC,
    encR_pinA, encR_pinB, encR_pinC;

    double 
    wD,     // Wheel Diameter (Def: 150)
    wL,     // Wheel Lenght From Center Left To Center Right (Def: 413)
    wC,     // Wheel Circum
    wRes;   // Wheel Move Resolution

    int8_t
    wL_dir, // Left Wheel Direction
    WR_dir; // Right Wheel Direction

  public:
    double
    dLeft, dRight, dCenter, 
    x, y, theta;
    
    int
    wl_counter,
    wr_counter;

    double
    lspeed, 
    rspeed;


    Odometry(const double &input_diameter, const double &input_length, const uint8_t input_encRes);
    
    void enc_Pinset(const uint8_t input_LpinA, const uint8_t input_LpinB, const uint8_t input_LpinC, const uint8_t input_RpinA, const uint8_t input_RpinB, const uint8_t input_RpinC);
    void enc_Read(void);
    void enc_Reset(void);

    void est_Speed(void);

    void get_Pos(void);
    void reset_Pos(void);
    void goTogoal(const double &inputX, const double &inputY, const double &inputTheta);
};

#endif