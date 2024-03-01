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
motor tlMotor18 = motor(PORT18, ratio6_1, true);
motor mlMotor19 = motor(PORT19, ratio6_1, true);
motor blMotor20 = motor(PORT20, ratio6_1, true);
motor trMotor13 = motor(PORT13, ratio6_1, false);
motor mrMotor12 = motor(PORT12, ratio6_1, false);
motor brMotor11 = motor(PORT11, ratio6_1, false);
// Global Motor Definitions End

// Define Motor Groups Start
motor_group leftMotors = motor_group(tlMotor18, mlMotor19, blMotor20);
motor_group rightMotors = motor_group(trMotor13, mrMotor12, brMotor11);
// Define Motor Groups End

// Define Other Devices Start
inertial inertia5 = inertial(PORT5);
controller driver = controller(primary);
// Define Other Devices End

// Global Variables Start
bool enabledrivePID = false;
float desiredDistance = 0.0;
float desiredAngle = 0.0;
float wheelDiameter = 3.25;
float gearRatio = .6;
float rotationScale = 360;
float wheelCircumference = M_PI * wheelDiameter;

// PID Tuning Values Start 
float kP = 1.5;
float kI = 0.005;
float kD = 0.4;

float tkP = 0.4;
float tkI = 0.04;
float tkD = 3.0;
// PID Tuning Values End

// Error and Derivative Values Start
float error = 0.0;
float preverror = 0.0;
float totalerror = 0.0;
float derivative = 0.0;

float turnerror = 0.0;
float turnpreverror = 0.0;
float turntotalerror = 0.0;
float turnderivative = 0.0;
// Error and Derivative Values End

// Antiintegral Wind-Up Values Start
float lateralI = 5;
float turnI = 15;
// Antiintegral Wind-Up Values End

// Tolerance Values Start
float toleranceLateral = 2;
float toleranceTurn = 1.5;
// Tolerance Values End

// Global Variables End

// pass this your desired PID tuning values for the lateral PID
void setlaterKvalues(float nkP, float nkI, float nkD){
   kP = nkP;
   kI = nkI;
   kD = nkD;
}

// same thing but for turn values
void setturnKvalues(float ntkP, float ntkI, float ntkD){
   tkP = ntkP;
   tkI = ntkI;
   tkD = ntkD;
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

bool pid_active = false;
// a task designed to control the drivetrain via PID takes no input.
int drivePID() { 
   bool rotationComplete = false;
   bool lateralComplete = false;
   while (enabledrivePID) {

      if (rotationComplete == true && lateralComplete == true) {
         pid_active = false;
         rightMotors.stop(hold);
         leftMotors.stop(hold);
      } else if(pid_active == true){
         rotationComplete = false;
         lateralComplete = false;
      }

      // Motor Positions Start
      float leftPosition = leftMotors.position(degrees);
      // Get the mean position of motors on the left side of the drivetrain
      float rightPosition = rightMotors.position(degrees);

      float averagePosition = (leftPosition + rightPosition) / 2; // get the average position of both sides of the drive train
      // Motor Position End

      // Lateral PID Start
      error = desiredDistance - averagePosition;

      derivative = error - preverror;

      if (error < lateralI) {
         totalerror += error;
      }

      preverror = error;

      // If you are within error bounds register as complete
      if (fabs(error) < toleranceLateral) {
         lateralComplete = true;
      }


      /*
      Error is the difference between the position you want to be and where you currently are ie how far you have left to go
      Derivative is the difference between your current distance and your previous distance ie how far you have traveled in one cycle
      Total Error is your compounding error ie how far you have traveled note that this only runs within certain bounds to prevent windup

      We multiply these values by our PID tuning values to acquire a value that specifies the voltage to move the motors at.
      thus giving us our lateral motor power or LMP
      */

      float lateralmotorPower = ((error * kP) + (derivative * kD) + (totalerror * kI)) / 12.7; //LMP
      printf("LMP %f\n", lateralmotorPower);
      // Lateral PID End

      // Rotational PID Start
      /*
      This is slightly more complex as it requires the use of an inertial sensor but has the capacity to operate without it.
      It is recommended that you use an inertial sensor.
      Outside of that, it is exactly the same as before.
      */
      
      // borrowed from JAR-Template however it just gets the absolute of its current position
   
      float currentHeading = reduce_0_to_360(inertia5.rotation()*360.0/rotationScale);;

      turnerror = reduce_negative_180_to_180(desiredAngle - currentHeading);
      
      if(fabs(turnerror) <= toleranceTurn && fabs(error) <= toleranceLateral) {
         turnerror = 0;
         turnderivative = 0;
         turntotalerror = 0;
         rotationComplete = true;
         error = 0;
         derivative = 0;
         totalerror = 0;
      }

      turnderivative = turnerror - turnpreverror;

      if (turnerror < turnI) {
         turntotalerror += turnerror;
      }

      turnpreverror = turnerror;

      float turnmotorPower = ((turnerror * tkP) + (turnderivative * tkD) + (turntotalerror * tkI)) / 12.7; // TMP
      printf("TMP %f\n", turnmotorPower);
      // Rotational PID End
      
      rightMotors.spin(forward, (lateralmotorPower - turnmotorPower), volt);
      leftMotors.spin (forward, (lateralmotorPower + turnmotorPower), volt);

      task::sleep(50); // time between updates to the PID loop.
      
   }

   return(1);
}

void prepSys() {
   // Preamble that tells the robot it is where it should be to start
   inertia5.calibrate();
   leftMotors.resetPosition();
   rightMotors.resetPosition();
   inertia5.setHeading(0, deg);
   inertia5.setRotation(0, deg);
   leftMotors.setPosition(0, deg);
   rightMotors.setPosition(0, deg);
}

void move(float distance, float angle) {
// convert linear distance to angular distance
   while (pid_active == true) {
      wait(5, msec);
   }
   if (pid_active == false){
      float degreesWanted = 2*((distance*(360*gearRatio))/wheelCircumference);
      desiredDistance = degreesWanted;

      desiredAngle = angle;
   }
}

int main() {
   prepSys();
   enabledrivePID = true;
   task PID( drivePID );
   move(48.0, 90.0);
   // pray it works

}
