#include "vex.h"

using namespace vex;

class pid {
    int Timeout;
        
    int settleTime;
            
    float integral;
           
    float derivative;
          
    float preverror;

    // p tuning constant
    float kP;

    // i tuning constant
    float kI;

    // d tuning constant
    float kD;

    // value at which integral value starts compounding
    float aiwValue;
    
    float settleBounds;

    int runningtime = 0;

    int settledtime = 0;

    // maximum voltage
    float max = 12.7;

    public:
        pid(float kp, float ki, float kd, float aiwvalue, int timeout, int settletime, float settlebounds, float Max){
           kP = kp;
           kI = ki;
           kD = kd;
           aiwValue = aiwvalue;

           if ((timeout % 10) >= 5) {
               Timeout = timeout + (10 - (timeout % 10));
           } else if ((timeout % 10) >= 1) {
               Timeout = timeout - (timeout % 10);
           } else {
             Timeout = timeout;   
           }

           if ((settletime % 10) >= 5) {
               settleTime = settletime + (10 - (settletime % 10));
           } else if ((settletime % 10) >= 1) {
               settleTime = settletime - (settletime % 10);
           } else {
             settleTime = settletime;   
           }

           settleBounds = settlebounds;

           max = Max;
        }
        
        // ensure voltage mins and maxes on inputVoltage
        float calculateoutput(float inputVoltage) {

            if (inputVoltage < -max) {
                return -max;
            } else if (inputVoltage > max) {
                return max;
            }

            return inputVoltage;
        }

        // calculate pid voltage
        float calcPID(float error) {
            
            if (fabs(error) < aiwValue) {
                integral += error;
            }

            if ((error < 0 && preverror > 0) || (error > 0 && preverror < 0)) {
                integral = 0;
            }

            derivative = preverror - error;

            float rawvalue = ((kP*error) + (kI * integral) + (kD * derivative)) / 12.7;

            preverror = error;

            if (fabs(error) < settleBounds) {
                settledtime += 10;
            } else {
                settledtime = 0;
            }

            runningtime += 10;
            
            return calculateoutput(rawvalue);
        }

        bool active() {
            if (Timeout != 0 && runningtime >= Timeout) {
                return false;
            }

            if (settledtime >= settleTime) {
                return false;
            }

            return true;
        }
};