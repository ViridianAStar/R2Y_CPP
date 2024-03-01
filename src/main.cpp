/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       24_nyocum                                                 */
/*    Created:      2/29/2024, 3:38:22 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "clock.cpp"

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

// Other Objects Start
clockD clock = clockD();

// Global Variables Start
bool enabledrivePID = false;
double desiredDistance = 0.0;
double desiredAngle = 0.0;
double wheelDiameter = 3.25;
double gearRatio = 0.6;
double wheelCircumference = M_PI * wheelDiameter;
// Global Variables End

// a task designed to control the drivetrain via PID takes no input.
int drivePID() {
 // PID Tuning Values Start 
    double kP = 1.5;
    double kI = 0.005;
    double kD = 10.0;

    double tkP = 0.4;
    double tkI = 0.04;
    double tkD = 3.0;
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

      double averagePosition = (leftPosition + rightPosition) / 2; // get the average position of both sides of the drive train
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

      double currentAngle = inertia5.heading();

      turnerror = desiredAngle - currentAngle;

      turnderivative = turnerror - turnpreverror;

      turntotalerror += turnerror;

      turnpreverror = turnerror;

      double turnmotorPower = ((turnerror * tkP) + (turnderivative * tkD) + (turntotalerror * tkI)) / 12.7; // TMP
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

void move(double distance/*, double angle*/) {
   clock.start();
   double degreesWanted = ((distance/wheelCircumference)*360.0)*gearRatio;
   desiredDistance = degreesWanted;
}

int main() {
   prepSys();
   enabledrivePID = true;
   task PID( drivePID );
   move(12.0);
   clock.stop();
   printf("Time Taken: %f ms\n", clock.timeTaken);
   // pray it works

}
