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
// Global Motor Definitions End

// Define Motor Groups Start
motor_group leftMotors = motor_group();
motor_group rightMotors = motor_group();
// Define Motor Groups End

// Define Other Devices Start
inertial inertia5 = inertial(PORT5);
// Define Other Devices End

// Global Variables Start
bool enabledrivePID = false;
double desiredDistance = 0.0;
double desiredAngle = 0.0;
double wheelDiameter = 0.0;
double gearRatio = 0.0;
double wheelCircumference = M_PI * wheelDiameter;
// Global Variables End

// a task designed to control the drivetrain via PID
int drivePID() {
 // PID Tuning Values Start 
    double kP = 0.1;
    double kI = 0.0005;
    double kD = 0.0;

    double tkP = 0.0;
    double tkI = 0.0;
    double tkD = 0.0;
 // PID Tuning Values End

 // Error and Derivative Values Start
    double error = 0.0;
    double preverror = 0.0;
    double totalerror = 0.0;
    double derivative = 0.0;

    double turnerror = 0.0;
    double turnpreverror = 0.0;
    double turntotalerror = 0.0;
    double turnderivative = 0.0;
 // Error and Derivative Values End
 
   while (enabledrivePID) {

      // Motor Positions Start
      double leftPosition = leftMotors.position(degrees);
      // get the mean position of motors on the left side of the drivetrain
      double rightPosition = rightMotors.position(degrees);

      double averagePosition = (leftPosition+rightPosition)/2; // get the average position of both sides of the drive train
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

      double lateralmotorPower = ((error * kP) + (derivative * kD) + (totalerror * kI)) / 12.7; //LMP
      // Lateral PID End

      // Rotational PID Start
      /*
      This is slightly more complex as it requires the use of an inertial sensor but has the capacity to operate without it.
      It is recommended that you use an inertial sensor.
      Outside of that, it is exactly the same as before.
      */

      double currentAngle = inertia5.heading();

      turnerror = desiredAngle - currentAngle;

      turnderivative = turnerror - turnpreverror;

      turntotalerror += turnerror;

      turnpreverror = turnerror;

      double turnmotorPower = ((turnerror * tkP) + (turnderivative * tkD) + (turntotalerror * tkI)) / 12.7;
      // Rotational PID End

      rightMotors.spin(forward, (lateralmotorPower - turnmotorPower), volt);
      leftMotors.spin (forward, (lateralmotorPower + turnmotorPower), volt);

      task::sleep(50); // time between updates to the PID loop.
   }

   return(1);
}

int main() {
   while (1){}
}
