#include "vex.h"

PID::PID(float error, float integral, float derivative, float kP, float kI, float kD, float toleranceValue, float aiwValue, float max, float min) :
   error(&error),
   integral(&integral),
   derivative(&derivative),
   kP(&kP),
   kI(&kI),
   kD(&kD),
   toleranceValue(&toleranceValue),
   aiwValue(&aiwValue),
   max(&max),
   min(&min)
{};

// a task designed to control the drivetrain via PID takes no input.
float PID::sysPIDcompute(float error, float max, float min) {
   // if within bounds add error to your integral value
   if (fabs(error) < aiwValue) {
      integral += error;
   }
   // if your error is beyond 0 eliminates integral
   if ((error<0 && preverror>0) || (error>0 && preverror<0)){
      integral = 0;
   }
   derivative = error - preverror;
   float driveoutput = ((kP * error) + (kI * integral) + (kD * derivative));
   if (fabs(error) <= toleranceValue) {
      timeinactive += 10;
   } else {
      timeinactive = 0;
   }
   timerunning += 10;
   preverror = error;
   if (driveoutput > max) {
      driveoutput = max;
   } else if (driveoutput < min) {
      driveoutput = min;
   }
   return (driveoutput);
}

// Checks if PID is active and running
bool PID::isactivePID() {
   if (timerunning > timeout && timeout != 0) {
      return (false);
   } else if (timeinactive >= timetosettle) { 
      return (false);
   } else {
      return (true);
   }
}