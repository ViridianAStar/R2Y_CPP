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
distance leftdistanceSensor = distance(PORT5);
distance centerdistanceSensor = distance(PORT6);
distance rightdistanceSensor = distance(PORT7);
controller driver = controller(primary);
// Define Other Devices End

// Global Variables Start
float wheelDiameter = 2.75;
float gearRatio = 1.0;
float wheelbase = 6.5;
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
  .25, 0.005, 0.2,  // kP, kI, kD

  // pass this your rotational PID tuning values
  0.95, 0.0085, 0.64, // kP, kI, kD

  // pass this your swing PID tuning values
  3.8, 0.0055, .075, // kP, kI, kD

  // pass this your moving swing PID tuning values
  1.5, 0.3, 0.7, // kP, kI, kD

  // pass this your timeout values (timeout, settle time)
  10000, 150, 

  // pass this your voltage max/min values (rotational, swing, moving swing, lateral)
  8, 9, 11, 11,

  // pass this your settle bounds (lateral, rotational, swing, moving swing)
  5, 1, 1, 1,

  // pass this your anti integral windup bounds (lateral, rotational, swing, moving swing)
  41, 10, 10, 35,

  // Distance between wheel sides
  wheelbase

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

void widecurveLeft(float angle, float sidelength, int iterations) {
   driveControl.set_swing_tuning((3.8/2.5), 0.0055, .075);
   for (int i = 0; i < iterations; i++) {
      driveControl.move_distance(sidelength);
      driveControl.swing_towards_angle_right(driveControl.reduce_0_to_360(rotationalSensor.rotation()) - angle);
   }
}

bool checking = false;

int where_is_error() {

   while (checking == true) {

      if (leftdistanceSensor.objectDistance(mm) <= 105) {

         driveControl.triggerInterupt();
         driveControl.point_at_angle(rotationalSensor.rotation() + 5);

      } else if (rightdistanceSensor.objectDistance(mm) <= 105 ) {

         driveControl.triggerInterupt();
         driveControl.point_at_angle(rotationalSensor.rotation() - 5);

      } else if (centerdistanceSensor.objectDistance(mm) <= 155) {

         if (rightdistanceSensor.objectDistance(mm) < leftdistanceSensor.objectDistance(mm)) {

         driveControl.triggerInterupt();
         driveControl.point_at_angle(rotationalSensor.rotation() - 5);

         } else if (rightdistanceSensor.objectDistance(mm) > leftdistanceSensor.objectDistance(mm)){

         driveControl.triggerInterupt();
         driveControl.point_at_angle(rotationalSensor.rotation() + 5);

         } else if ((rightdistanceSensor.objectDistance(mm) <= leftdistanceSensor.objectDistance(mm) + 5 && rightdistanceSensor.objectDistance(mm) >= leftdistanceSensor.objectDistance(mm) - 5) && (leftdistanceSensor.objectDistance(mm) <= rightdistanceSensor.objectDistance(mm) + 5 && leftdistanceSensor.objectDistance(mm) >= rightdistanceSensor.objectDistance(mm) - 5)) {
            
            int validated_paths[2] = {0, 0};
            float path_weights[2] = {0.0, 0.0};

            driveControl.triggerInterupt();
            driveControl.point_at_angle(rotationalSensor.rotation() + 45);

            float p0R = rightdistanceSensor.objectDistance(mm);
            float p0C = centerdistanceSensor.objectDistance(mm);
            float p0L = leftdistanceSensor.objectDistance(mm);
            float ph0 = rotationalSensor.rotation();

            driveControl.point_at_angle(rotationalSensor.rotation() + 90);

            float p1R = rightdistanceSensor.objectDistance(mm);
            float p1C = centerdistanceSensor.objectDistance(mm);
            float p1L = leftdistanceSensor.objectDistance(mm);
            float ph1 = rotationalSensor.rotation();

            if (p0C > 150) {
               if (p0L <= 105 && p0R <= 105) {
                  validated_paths[0] = 0;
               } else if (p0L <= 105 && p0R > 105) {
                  validated_paths[0] = 1;
                  if (p0L <= 45) {
                     path_weights[0] = 0.1;
                     ph0 = ph0 + 25;
                  } else if (p0L > 45 && p0L <= 60) {
                     path_weights[0] = 0.25;
                     ph0 = ph0 + 20;
                  } else if (p0L > 60 && p0L <= 75) {
                     path_weights[0] = 0.45;
                     ph0 = ph0 + 15;
                  } else if (p0L > 75 && p0L <= 90) {
                     path_weights[0] = 0.7
                     ph0 = ph0 + 10;
                  } else if (p0L > 90 && p0L <= 105) {
                     path_weights[0] = 0.85; 
                     ph0 = ph0 + 5;
                  }
               } else if (p0L > 105 && p0R <= 105) {
                  validated_paths[0] = 1;
                  if (p0L <= 45) {
                     path_weights[0] = 0.1;
                     ph0 = ph0 - 25;
                  } else if (p0L > 45 && p0L <= 60) {
                     path_weights[0] = 0.25;
                     ph0 = ph0 - 20;
                  } else if (p0L > 60 && p0L <= 75) {
                     path_weights[0] = 0.45;
                     ph0 = ph0 - 15;
                  } else if (p0L > 75 && p0L <= 90) {
                     path_weights[0] = 0.7
                     ph0 = ph0 - 10;
                  } else if (p0L > 90 && p0L <= 105) {
                     path_weights[0] = 0.85; 
                     ph0 = ph0 - 5;
                  }
               }
            } 

         }

      }

      task::sleep(10);
   }
 
   return(-1);
}

void self_guidance() {

}

int main() {
   prepSys();

   
   /*driveControl.move_distance(190);
   for (int i = 0; i <= 2; i++) {
      driveControl.move_distance(-50);
      wait(50, msec);
      driveControl.move_distance(50);
   }
   */
   brakemode(brake);
   while (1) {
      leftMotors.spin(forward, (driver.Axis3.value() + driver. Axis1.value())/10, volt);
      rightMotors.spin(forward, (driver.Axis3.value() - driver.Axis1.value())/10, volt);
   }
   
}

