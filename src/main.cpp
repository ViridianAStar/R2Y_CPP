/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       24_nyocum                                                 */
/*    Created:      2/29/2024, 3:38:22 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "movement.cpp"

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

movement driveControl = movement(leftMotors, rightMotors, gearRatio, float(3.25), float(.5), float(0.005), float(0.7), float(0.8), float(0.008), float(0.5), float(0.75), float(0.0075), float(0.5), 1000, 150, float(10), float(9), float(10.7));

// Global Variables End 

void prepSys() {
   // Preamble that tells the robot it is where it should be to start
   inertia14.calibrate(); // calibrate for accuracy
   leftMotors.resetPosition(); // reset their postion
   rightMotors.resetPosition(); // reset their position
   inertia14.setHeading(0, deg); // tell it what its rotation is
   inertia14.setRotation(0, deg); // tell it what its rotation is (on xy plane)
   leftMotors.setPosition(0, deg); // tell them they are at 0
   rightMotors.setPosition(0, deg); // tell them they are at 0
   while (inertia14.isCalibrating() == true){
      wait(5, msec);
   }
}

void square() {
   driveControl.move_distance(20, 0);
   driveControl.swing_towards_angle_right(90);
   driveControl.move_distance(20, 90);
   driveControl.swing_towards_angle_right(180);
   driveControl.move_distance(20, 180);
   driveControl.swing_towards_angle_right(270);
   driveControl.move_distance(20, 270);
   driveControl.swing_towards_angle_right(0);
}

void brakemode(brakeType mode) {
   leftMotors.setStopping(brake);
   rightMotors.setStopping(brake);
}

void userDrive() {
   leftMotors.spin(forward, ((driver.Axis2.value() + driver.Axis3.value())/10), volt);
   rightMotors.spin(forward, ((driver.Axis2.value() - driver.Axis3.value())/10), volt);
}

int main() {
   prepSys();
   //square();

   /*while (1) {
      userDrive();
   }*/
   
}

