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
// Define Other Devices End

// Global Variables Start
bool enabledrivePID = false;
float desiredDistance = 0.0;
float desiredAngle = 0.0;
float wheelDiameter = 3.25;
float gearRatio = 0.6;
float wheelCircumference = M_PI * wheelDiameter;
// Global Variables End

// a task designed to control the drivetrain via PID takes no input.
int drivePID() {
 // PID Tuning Values Start 
    float kP = 1.5;
    float kI = 0.005;
    float kD = 10.0;

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
 
   while (enabledrivePID) {
      // Motor Positions Start
      float leftPosition = leftMotors.position(degrees);
      // get the mean position of motors on the left side of the drivetrain
      float rightPosition = rightMotors.position(degrees);

      float averagePosition = (leftPosition + rightPosition) / 2; // get the average position of both sides of the drive train
      // Motor Position End

      // Lateral PID Start
      error = desiredDistance - averagePosition;

      derivative = error - preverror;

      totalerror += error;

      preverror = error;


      /*
      Error is the difference between the position you want to be and where you currently are ie how far you have left to go
      Derivative is the difference between your current distance and your previous distance ie how far you have traveled in one cycle
      Total Error is your compounding error ie how far you have traveled

      We multiply these values by our PID tuning values to acquire a value that specifies the voltage to move the motors at.
      thus giving us our lateral motor power or LMP
      */

      float lateralmotorPower = ((error * kP) + (derivative * kD) + (totalerror * kI)) / 12.7; //LMP
      printf("kP %f\n", kP);
      printf("kI %f\n", kI);
      printf("kD %f\n", kD);
      printf("e %f\n", error);
      printf("te %f\n", totalerror);
      printf("d %f\n", derivative);
      printf("kPe %f\n", (error * kP));
      printf("kIe %f\n", (totalerror * kI));
      printf("kDe %f\n", (derivative * kD));
      printf("LMP %f\n", lateralmotorPower);
      // Lateral PID End

      // Rotational PID Start
      /*
      This is slightly more complex as it requires the use of an inertial sensor but has the capacity to operate without it.
      It is recommended that you use an inertial sensor.
      Outside of that, it is exactly the same as before.
      */

      float currentHeading = inertia5.heading();

      turnerror = desiredAngle - currentHeading;

      turnderivative = turnerror - turnpreverror;

      turntotalerror += turnerror;

      turnpreverror = turnerror;

      float turnmotorPower = ((turnerror * tkP) + (turnderivative * tkD) + (turntotalerror * tkI)) / 12.7; // TMP
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

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

void move(float distance, float angle) {
   float degreesWanted = (distance*(360*gearRatio))/wheelCircumference;
   desiredDistance = degreesWanted;
   
   desiredAngle = angle;
}

int main() {
   prepSys();
   enabledrivePID = true;
   task PID( drivePID );
   move(12.0, 0.0);
   // pray it works

}
