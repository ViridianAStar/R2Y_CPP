#pragma once
#include "vex.h"

class movement {
    public:
        float lkP;
        float lkI;
        float lkD;  
        float rkP;
        float rkI;
        float rkD;  
        float skP;
        float skI;
        float skD;  
        float lsettleBounds = 5;
        float rsettleBounds = 5;
        float ssettleBounds = 5;
        int settleTime;
        int Timeout;    
        float laiwValue = 20;
        float raiwValue = 15;
        float saiwValue = 15;   
        float tmv;
        float smv;
        float lmv;  
        float gearRatio;
        float circumference;
        motor_group leftside;
        motor_group rightside;

        movement(motor_group left, motor_group right, float gearratio, float wheeldiameter, float lkp, float lki, float lkd, float rkp, float rki, float rkd, float skp, float ski, float skd, int timeout, int settletime, float TMV, float SMV, float LMV);
        void move_distance(float distance, float desired_heading);
        void point_at_angle(float angle);
        void swing_towards_angle_left(float angle);
        void swing_towards_angle_right(float angle);
        float reduce_negative_90_to_90(float angle);
        float reduce_negative_180_to_180(float angle);
        float reduce_0_to_360(float angle);

};