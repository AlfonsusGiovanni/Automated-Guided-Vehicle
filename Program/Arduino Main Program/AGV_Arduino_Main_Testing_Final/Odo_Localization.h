/*  
  AGV ODOMETRY LOCALIZATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 11 November 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#ifndef ODO_LOCALIZATION_H
#define ODO_LOCALIZATION_H

#include "Arduino.h"
#include "math.h"

class Odometry{
  public:
    uint8_t
    encL_pinA, encL_pinB, encL_pinC,
    encR_pinA, encR_pinB, encR_pinC;

    double 
    wD,     // Wheel Diameter (Def: 150)
    wL,     // Wheel Lenght From Center Left To Center Right (Def: 413)
    wC,     // Wheel Circum
    wRes,   // Wheel Move Resolution

    Left_dist,
    Right_dist,

    start_heading_angle,
    goal_heading_angle,
    current_heading_angle,

    Adist, Bdist, Cdist,

    x, y, theta,
    x1, y1, theta1;
    
    int
    wl_counter,
    wr_counter;

    double
    lspeed, 
    rspeed;

    // Odometry Initialize
    Odometry(const double &input_diameter, const double &input_length, const uint8_t input_encRes);
    
    // Odometry Pin Set
    void enc_Pinset(const uint8_t input_LpinA, const uint8_t input_LpinB, const uint8_t input_LpinC, const uint8_t input_RpinA, const uint8_t input_RpinB, const uint8_t input_RpinC);
    
    // Encoder Read Function
    void enc_Read(void);

    // Encoder Reset Counter
    void enc_Reset(void);

    // Estimate Current Speed
    void est_Speed(void);

    // Reset Heading Angle
    void reset_Head(void);

    // Get Current Pos
    void get_Pos(void);

    // Reset Current Pos
    void reset_Pos(void);

    // Calculate PWM Output
    void pwm_Out(void);

    bool goTogoal(const double &inputX, const double &inputY, const double &inputTheta);
};

#endif