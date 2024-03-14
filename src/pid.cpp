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
            
            // first we check if our error is less than your anti integral windup value, if it is
            // we start compounding.
            if (fabs(error) < aiwValue) {
                integral += error;
            }

            // if your error crosses 0 eliminate your integral
            if ((error < 0 && preverror > 0) || (error > 0 && preverror < 0)) {
                integral = 0;
            }

            // your derivitave is how far away you were last cycle minus how far away you are this cycle.
            derivative = preverror - error;

            // to calculate our raw power/voltage value we multiply our error derivative and integral values by their respective
            // tuning values then divide by 12.7 as we are working with motors that go up to 12.7 volts.
            float rawvalue = ((kP*error) + (kI * integral) + (kD * derivative)) / 12.7;
            
            // set our previous error for our next run
            preverror = error;

            // check to see if we are withing settle bounds if yes add time if no set time to 0
            if (fabs(error) < settleBounds) {
                settledtime += 10;
            } else {
                settledtime = 0;
            }

            // add running time for timeout
            runningtime += 10;
            
            // return the output ensuring its value is within your voltage limits
            return calculateoutput(rawvalue);
        }

        bool active() {
            // if your timeout is reached and not 0 as 0 would be infinite you are complete
            if (Timeout != 0 && runningtime >= Timeout) {
                return false;
            }
            
            // if your settled time is equal to or greater than your settle time you are complete
            if (settledtime >= settleTime) {
                return false;
            }

            // otherwise you are not.
            return true;
        }
};