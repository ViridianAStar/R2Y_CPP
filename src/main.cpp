/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       24_nyocum                                                 */
/*    Created:      2/29/2024, 3:38:22 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "movement.h"

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
// DO NOT RENAME THE INERTIAL/ROTATIONAL SENSOR OBJECT UNLESS YOU CHANGE THE EXTERN IN vex.h
inertial rotationalSensor = inertial(PORT14);
controller driver = controller(primary);
// Define Other Devices End

// Global Variables Start
float desiredDistance = 0.0;
float desiredAngle = 0.0;
float desiredswingAngle = 0.0;
float wheelDiameter = 3.25;
float gearRatio = .6;



// Global Variables End 

// Drive Control Initialization Start
movement driveControl = movement(
  // pass this your left motor and right motor groups
  leftMotors,
  rightMotors, 
  //pass this your gear ratio and wheel diameters
  gearRatio, 
  wheelDiameter, 
  // pass this your lateral PID tuning values
  1.5, 0.005, 0.7,

  // pass this your rotational PID tuning values
  0.8, 0.008, 0.5,

  // pass this your swing PID tuning values
  3.8, 0.055, .75,

  // pass this your timeout values (timeout, settle time)
  1000, 150, 

  // pass this your voltage max/min values (lateral, rotational, swing)
  11, 9, 10
  );
// Drive Control Initialization Start

void prepSys() {
   // Preamble that tells the robot it is where it should be to start
   rotationalSensor.calibrate(); // calibrate for accuracy
   leftMotors.resetPosition(); // reset their postion
   rightMotors.resetPosition(); // reset their position
   rotationalSensor.setHeading(0, deg); // tell it what its rotation is
   rotationalSensor.setRotation(0, deg); // tell it what its rotation is (on xy plane)
   leftMotors.setPosition(0, deg); // tell them they are at 0
   rightMotors.setPosition(0, deg); // tell them they are at 0
   while (rotationalSensor.isCalibrating() == true){
      wait(5, msec);
   }
}

void printvalues(int tval) {
   Brain.Screen.setCursor(5, 7);
   Brain.Screen.print(rotationalSensor.rotation());
   wait(tval, msec);
   Brain.Screen.clearScreen();
}

void square() {
   printvalues(10000);
   driveControl.move_distance(100);
   printvalues(10000);
   driveControl.swing_towards_angle_right(90);
   printvalues(10000);
   driveControl.move_distance(100);
   printvalues(10000);
   driveControl.swing_towards_angle_right(180);
   printvalues(10000);
   driveControl.move_distance(100);
   printvalues(10000);
   driveControl.swing_towards_angle_right(270);
   printvalues(10000);
   driveControl.move_distance(100);
   printvalues(10000);
   driveControl.swing_towards_angle_right(0);
   printvalues(10000);
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
   square();
   /*driveControl.swing_towards_angle_left(90);
   printvalues(10000);
   driveControl.swing_towards_angle_left(180);
   printvalues(10000);
   driveControl.swing_towards_angle_left(270);
   printvalues(10000);
   driveControl.swing_towards_angle_left(0);
   printvalues(10000);*/
   /*while (1) {
      userDrive();
   }*/
   
}

