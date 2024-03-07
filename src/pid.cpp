#include "vex.h"

using namespace vex;

// structure for lateral data
struct lateralpidData {
    // distance ie how far you want to go
    float distance;
    // what you want your heading to be
    float angle;
    // your encoder position currently
    float currentAvg;

    float timeout;

    float settletime;

    float integral;
    
    float derivative;
    
    float preverror;
};

struct generalPIDData {
    // maximum voltage
    float max;
    // minimum voltage
    float min;
    // lateral starti (anti-integral windup)
    float lateralstarti;
    // rotational starti
    float rotationstarti;
    // swing starti
    float swingstarti;

    // pid constants
    float lkP = .4;
    float lkI = 0.005;
    float lkD = 0.2;

    float tkP = .2;
    float tkI = 0.04;
    float tkD = 0.08;

    float skP = 0.3;
    float skI = 0.001;
    float skD = 2;
    
};

// initialize lateraldesireds
lateralpidData lateraldesireds;

generalPIDData genPID;

void getpidData(float angle, float distance, float currentAvg) {
    lateraldesireds.angle = angle;
    lateraldesireds.distance = distance+currentAvg;
    lateraldesireds.currentAvg = currentAvg;
}

float lateralPID() {

    float error = (lateraldesireds.distance - lateraldesireds.currentAvg);

    if (fabs(error) < genPID.lateralstarti) {
        lateraldesireds.integral += error;
    }

    if ((lateraldesireds.derivative < 0 && error > 0) ||(lateraldesireds.derivative > 0 && error < 0)) {
        lateraldesireds.integral = 0;
    }

    lateraldesireds.derivative = error - lateraldesireds.preverror;
        
    lateraldesireds.preverror = error;

    float power = ((genPID.lkP * error) + (genPID.lkI * lateraldesireds.integral) + (genPID.lkD * lateraldesireds.derivative))/12.7;

    if (power < genPID.min) {
        power = genPID.min;
    } else if (power > genPID.max) {
        power = genPID.max;
    }

    return power;
}

bool lateralActive() {
    if (lateraldesireds.timeout) {

    }

}

float headingPID() {


    return 0;
}