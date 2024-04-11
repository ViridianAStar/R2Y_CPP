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

        float mskP;
        float mskI;
        float mskD;

        float lsettleBounds = 5;
        float rsettleBounds = 5;
        float ssettleBounds = 5;
        float mssettleBounds = 5;

        int settleTime;
        int Timeout;    

        float laiwValue = 20;
        float raiwValue = 15;
        float saiwValue = 15;
        float msaiwValue = 15;   

        float lmv; 
        float tmv;
        float smv; 
        float msmv;

        float gearRatio;
        float circumference;
        float WheelBase;
        motor_group leftside;
        motor_group rightside;

        bool interupt = false;

        movement(motor_group left, motor_group right, float gearratio, float wheeldiameter, float lkp, float lki, float lkd, float rkp, float rki, float rkd, float skp, float ski, float skd, float mskp, float mski, float mskd, int timeout, int settletime, float TMV, float SMV, float MSMV, float LMV, float LaiwValue, float RaiwValue, float SaiwValue, float MSaiwValue, float LsettleBounds, float RsettleBounds, float SsettleBounds, float MSsettleBounds, float wheelBase);
        
        void move_distance(float distance);
        void movingSwingleft(float  precision, float radius, float angle);
        void movingSwingright(float precision, float radius, float angle);
        void point_at_angle(float angle);
        void swing_towards_angle_left(float angle);
        void swing_towards_angle_right(float angle);
        void set_lateral_tuning(float p, float i, float d);
        void set_rotational_tuning(float p, float i, float d);
        void set_swing_tuning(float p, float i, float d);
        void set_timeout(int t);
        void set_settle_time(int t);
        void set_voltage_limits(float LMV, float TMV, float SMV);

        void triggerInterupt();

        float reduce_negative_90_to_90(float angle);
        float reduce_negative_180_to_180(float angle);
        float reduce_0_to_360(float angle);

    class tankDrive {
    public: 
        motor_group TD_leftside;
        motor_group TD_rightside;
        tankDrive(motor_group left, motor_group right);

        void right_handed_userDrive();
        void single_stick_right_handed_userDrive();
        void left_handed_userDrive();
        void single_stick_left_handed_userDrive();
    };

};