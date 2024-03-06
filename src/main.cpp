/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       24_nyocum                                                 */
/*    Created:      2/29/2024, 3:38:22 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// Global Motor Definitions Start
motor tlMotor01 = motor(PORT1, ratio6_1, false);
motor blMotor11 = motor(PORT11, ratio6_1, false);
motor trMotor10 = motor(PORT10, ratio6_1, true);
motor brMotor20 = motor(PORT20, ratio6_1, true);
// Global Motor Definitions End

// Define Motor Groups Start
motor_group leftMotors = motor_group(tlMotor01, blMotor11);
motor_group rightMotors = motor_group(trMotor10, brMotor20);
// Define Motor Groups End

// Define Other Devices Start
inertial inertia14 = inertial(PORT14);
controller driver = controller(primary);
// Define Other Devices End

// Global Variables Start
float desiredDistance = 0.0;
float desiredAngle = 0.0;
float desiredswingAngle = 0.0;
float wheelDiameter = 3.25;
float gearRatio = .6;
float rotationScale = 360;
float wheelCircumference = M_PI * wheelDiameter;
int timerunning = 0;
int timeout = 1250;
int timetosettle = 100;
int timeinactive = 0;

// PID Tuning Values Start 
float lkP = .4;
float lkI = 0.005;
float lkD = 0.2;

float tkP = .2;
float tkI = 0.04;
float tkD = 0.08;

float skP = 0.3;
float skI = 0.001;
float skD = 2;
// PID Tuning Values End

// Antiintegral Wind-Up Values Start
float lateralI = 15;
float turnI = 31;
float swingI = 2;
// Antiintegral Wind-Up Values End

// Tolerance Values Start
float toleranceLateral = 0.5;
float toleranceTurn = 0.5;
float toleranceSwing = 0.5;
// Tolerance Values End

// Voltage Limiters Start
float lateralMax = 12.7;
float turnMax = 12.7;
float swingMax = 12.7;
float lateralMin = .7;
float turnMin = .2;
float swingMin = .7;
// Voltage Limiters End

// Swing and PID Activity Tracking Values Start
bool activePID = false;
bool completedPID = false;
// Swing and PID Activity Tracking Values End

// Global Variables End

// pass this your desired PID tuning values for the lateral PID
void setlaterKvalues(float nkP, float nkI, float nkD){
   lkP = nkP;
   lkI = nkI;
   lkD = nkD;
}

// same thing but for turn values
void setturnKvalues(float ntkP, float ntkI, float ntkD){
   tkP = ntkP;
   tkI = ntkI;
   tkD = ntkD;
}

// same thing but for swing values
void setswingKvalues(float nskP, float nskI, float nskD){
   skP = nskP;
   skI = nskI;
   skD = nskD;
}

// mass edit voltage limiters
void setvoltageLimits(float nlvT, float ntvT, float nsvT) {
   lateralMax = nlvT;
   turnMax = ntvT;
   swingMax = nsvT;
}

// Functions borrowed from JAR-Template Start
float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}
// Functions borrowed from JAR-Template End

// PID Start


   PID::PID(float error, float integral, float derivative, float kP, float kI, float kD, float toleranceValue, float aiwValue, float max, float min) :
   error(error),
   integral(integral),
   derivative(derivative),
   kP(kP),
   kI(kI),
   kD(kD),
   toleranceValue(toleranceValue),
   aiwValue(aiwValue),
   max(max),
   min(min)
   {};

   // a task designed to control the drivetrain via PID takes no input.
   float PID::sysPIDcompute(float error) {
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

   bool PID::isactivePID() {
      if (timerunning > timeout && timeout != 0) {
         return (false);
      } else if (timeinactive >= timetosettle) { 
         return (false);
      } else {
         return (true);
      }
   }

// PID End

// Drive Control Start
// distance should be in inches but if it is not you can always add a modifier angle is in degrees by default
void movedistance(float distance) {
   // convert linear distance to angular distance
   
   // when PID is available for tasking, convert distance to degrees and/or tell it the desired angle to turn to and tell it that it is active
   float degreesWanted = (((distance*360)*gearRatio))/wheelCircumference;
   desiredDistance = degreesWanted;

   // dont worry about resetting
   float initialavgposition = (leftMotors.position(degrees) + rightMotors.position(degrees)) / 2;

   while (isactivePID() == true) {

   }

}

// true for left false for right
void point_to_angle(float angle) {

}

// Drive Control End

void prepSys() {
   // Preamble that tells the robot it is where it should be to start
   inertia14.calibrate(); // calibrate for accuracy
   leftMotors.resetPosition(); // reset their postion
   rightMotors.resetPosition(); // reset their position
   inertia14.setHeading(0, deg); // tell it what its rotation is
   inertia14.setRotation(0, deg); // tell it what its rotation is (on xy plane)
   leftMotors.setPosition(0, deg); // tell them they are at 0
   rightMotors.setPosition(0, deg); // tell them they are at 0
   wait(1850, msec);
}

int main() {
   prepSys();
   
}

