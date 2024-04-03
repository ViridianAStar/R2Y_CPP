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
// DO NOT RENAME THE INERTIAL/ROTATIONAL SENSOR OR PRIMARY CONTROLLER OBJECTS UNLESS YOU CHANGE THE EXTERN IN vex.h
inertial rotationalSensor = inertial(PORT14);
controller driver = controller(primary);
// Define Other Devices End

// Global Variables Start
float wheelDiameter = 2.75;
float gearRatio = 1.0;
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
  1.45, 0.0023, 0.7,  // kP, kI, kD

  // pass this your rotational PID tuning values
  0.8, 0.008, 0.5, // kP, kI, kD

  // pass this your swing PID tuning values
  3.8, 0.0055, .075, // kP, kI, kD

  // pass this your timeout values (timeout, settle time)
  10000, 150, 

  // pass this your voltage max/min values (lateral, rotational, swing)
  11, 9, 8,

  // pass this your settle bounds (lateral, rotational, swing)
  5, 1, 1,

  // pass this your anti integral windup bounds (lateral, rotational, swing)
  41, 10, 10

);

movement::tankDrive userControl = movement::tankDrive(leftMotors, rightMotors);
// Drive Control Initialization End

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
   printvalues(1000);
   driveControl.move_distance(24);
   printvalues(1000);
   driveControl.swing_towards_angle_left(90);
   printvalues(1000);
   driveControl.move_distance(24);
   printvalues(1000);
   driveControl.swing_towards_angle_left(180);
   printvalues(1000);
   driveControl.move_distance(24);
   printvalues(1000);
   driveControl.swing_towards_angle_left(270);
   printvalues(1000);
   driveControl.move_distance(24);
   printvalues(1000);
   driveControl.swing_towards_angle_left(0);
   printvalues(1000);
}

void triangle() {
   driveControl.move_distance(24);
   driveControl.swing_towards_angle_left(120);
   driveControl.move_distance(24);
   driveControl.swing_towards_angle_left(240);
   driveControl.move_distance(24);
   driveControl.swing_towards_angle_left(0);
}

void brakemode(brakeType mode) {
   leftMotors.setStopping(mode);
   rightMotors.setStopping(mode);
}

void cardinalswingTest() {
   Brain.Screen.setPenColor(green);
   driveControl.swing_towards_angle_left(90);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 90 / -270, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_left(180);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 180, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_left(270);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 270 / -90, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_left(0);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 0 / 360, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.move_distance(24);

   driveControl.swing_towards_angle_right(-90);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired -90 / 270, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_right(180);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired 180, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_right(-270);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired -270 / 90, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_right(0);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired 0 / 360, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();
}

void subcardinalswingTest() {
   Brain.Screen.setPenColor(green);
   driveControl.swing_towards_angle_left(45);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 45 / -45, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_left(135);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 135, -135, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_left(225);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 225 / -225, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_left(0);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("left desired 315 / -315, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.point_at_angle(0);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("Rotation Desired 0 / 360, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.move_distance(24);

   driveControl.swing_towards_angle_right(-45);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired -45 / 45, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_right(-135);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired -135, 135, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_right(-225);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired -225 / 225, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();

   driveControl.swing_towards_angle_right(-315);

   Brain.Screen.setCursor(6, 6);
   Brain.Screen.print("right desired -315 / 315, actual: ");
   Brain.Screen.print(driveControl.reduce_0_to_360(rotationalSensor.rotation()));
   wait(2, seconds);
   Brain.Screen.clearScreen();
}

void totalswingTest() {
   cardinalswingTest();
   wait(1, seconds);
   subcardinalswingTest();
}

void auton() {
   driveControl.move_distance(190);
   for (int i = 0; i <= 5; i++) {
      driveControl.move_distance(-50);
      wait(50, msec);
      driveControl.move_distance(50);
   }
}

int main() {
   prepSys();

   brakemode(brake);

   while (1) {
      driver.ButtonA.pressed(auton);
      leftMotors.spin(forward, (driver.Axis3.value() + driver. Axis1.value())/10, volt);
      rightMotors.spin(forward, (driver.Axis3.value() - driver.Axis1.value())/10, volt);
   }
   
}

