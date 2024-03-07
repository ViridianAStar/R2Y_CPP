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
    float integral;
    float derivative;
    float preverror;

    float error = (lateraldesireds.distance - lateraldesireds.currentAvg);

    if (fabs(error) < genPID.lateralstarti) {
        integral += error;
    }

    derivative = error - preverror;
        
    preverror = error;

    float power = ((genPID.lkP * error) + (genPID.lkI * integral) + (genPID.lkD * derivative))/12.7;

    return power;
}

bool lateralActive() {
    if (true) {

    }

}

float headingPID() {


    return 0;
}