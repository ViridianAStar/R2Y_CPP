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
bool enabledrivePID = false;
float desiredDistance = 0.0;
float desiredAngle = 0.0;
float desiredswingAngle = 0.0;
float wheelDiameter = 3.25;
float gearRatio = .6;
float rotationScale = 360;
float wheelCircumference = M_PI * wheelDiameter;

// PID Tuning Values Start 
float kP = .4;
float kI = 0.005;
float kD = 0.2;

float tkP = .2;
float tkI = 0.04;
float tkD = 0.08;

float skP = 0.3;
float skI = 0.001;
float skD = 2;
// PID Tuning Values End

// Error and Derivative Values Start
float error = 0.0;
float preverror = 0.0;
float totalerror = 0.0;
float derivative = 0.0;

float swingerror = 0.0;
float swingpreverror = 0.0;
float swingtotalerror = 0.0;
float swingderivative = 0.0;

float turnerror = 0.0;
float turnpreverror = 0.0;
float turntotalerror = 0.0;
float turnderivative = 0.0;
// Error and Derivative Values End

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
float lateralVoltagetotal = 12.7;
float turnVoltagetotal = 12.7;
float swingVoltagetotal = 12.7;
// Voltage Limiters End

// Swing and PID Activity Tracking Values Start
bool activePID = false;
bool turning = false;
bool isSwinging = false;
bool swingDirection = false;
// Swing and PID Activity Tracking Values End

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

// same thing but for swing values
void setswingKvalues(float nskP, float nskI, float nskD){
   skP = nskP;
   skI = nskI;
   skD = nskD;
}

// mass edit voltage limiters
void setvoltageLimits(float nlvT, float ntvT, float nsvT) {
   lateralVoltagetotal = nlvT;
   turnVoltagetotal = ntvT;
   swingVoltagetotal = nsvT;
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

// a task designed to control the drivetrain via PID takes no input.
int drivePID() { 
   bool rotationComplete = false;
   bool lateralComplete = false;
   float swingmotorPower = 0.0;
   float lateralmotorPower = 0;
   Brain.Screen.setPenColor(green);
   int cycles = 0;
   while (enabledrivePID) {

      if (rotationComplete == true && lateralComplete == true) {
         activePID = false;
         rightMotors.stop(hold);
         leftMotors.stop(hold);
         turning = false;
      } else if(activePID == true){
         rotationComplete = false;
         lateralComplete = false;
      }

      // Motor Positions Start
      float leftPosition = leftMotors.position(degrees);
      // Get the mean position of motors on the left side of the drivetrain
      float rightPosition = rightMotors.position(degrees);

      float averagePosition = (leftPosition + rightPosition) / 2; // get the average position of both sides of the drive train
      // Motor Position End

      if (isSwinging == false){

         if (turning == false) {
            // Lateral PID Start
            error = desiredDistance - averagePosition;
   
            derivative = error - preverror;
   
            if (fabs(error) < lateralI) {
               totalerror += error;
            }
   
            preverror = error;
   
            // If you are within error bounds register as complete
            if (fabs(error) <= toleranceLateral && fabs(error) >= -1*toleranceLateral) {
               lateralComplete = true;
            }
   
   
            /*
            Error is the difference between the position you want to be and where you currently are ie how far you have left to go
            Derivative is the difference between your current distance and your previous distance ie how far you have traveled in one cycle
            Total Error is your compounding error ie how far you have traveled note that this only runs within certain bounds to prevent windup
   
            We multiply these values by our PID tuning values to acquire a value that specifies the voltage to move the motors at.
            thus giving us our lateral motor power or LMP
            */
        
            lateralmotorPower = ((error * kP) + (derivative * kD) + (totalerror * kI)) / 12.7; //LMP
   
            // keep motor power within limits
            if (lateralmotorPower > lateralVoltagetotal) {
               lateralmotorPower = lateralVoltagetotal;
            } else if(lateralmotorPower < (lateralVoltagetotal * -1)) {
               lateralmotorPower = (lateralVoltagetotal * -1);
            }
   
            printf("LMP %f\n", lateralmotorPower);
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("LMP: ");
            Brain.Screen.print(lateralmotorPower);
   
            // Lateral PID End
         } else {
            lateralComplete = true;
         }
   
         // Rotational PID Start
         /*
         This is slightly more complex as it requires the use of an inertial sensor but has the capacity to operate without it.
         It is recommended that you use an inertial sensor.
         Outside of that, it is exactly the same as before.
         */
         
         // borrowed from JAR-Template however it just gets the absolute of its current position
      
         float currentHeading = reduce_0_to_360(inertia14.rotation(degrees)*360.0/rotationScale);
   
         printf("%f\n", currentHeading);
         Brain.Screen.setCursor(5, 1);
         Brain.Screen.print("CH: ");
         Brain.Screen.print(currentHeading);
   
         turnerror = reduce_negative_180_to_180(desiredAngle - currentHeading);
         
         // if both are true you are done moving if turn is true then you are done turning
         if((currentHeading <= (desiredAngle+toleranceTurn) && fabs(turnerror) >= (desiredAngle-toleranceTurn)) && (fabs(error) <= (toleranceLateral) && fabs(error) >= (-1*toleranceLateral))) {
            turnerror = 0;
            turnderivative = 0;
            turntotalerror = 0;
            rotationComplete = true;
            error = 0;
            derivative = 0;
            totalerror = 0;
            
         } else if(currentHeading <= (desiredAngle+toleranceTurn) && fabs(turnerror) >= (desiredAngle-toleranceTurn)) {
            turnerror = 0;
            turnderivative = 0;
            turntotalerror = 0;
            rotationComplete = true;
         } else if (currentHeading <= (desiredAngle+toleranceTurn) && fabs(turnerror) >= (desiredAngle-toleranceTurn) && turning == true) {
            turnerror = 0;
            turnderivative = 0;
            turntotalerror = 0;
            rotationComplete = true;
            rightMotors.stop(coast);
            leftMotors.stop(coast);
         }
   
         turnderivative = turnerror - turnpreverror;
   
         if (fabs(turnerror) < turnI) {
            turntotalerror += turnerror;
         }
   
         turnpreverror = turnerror;
   
         float turnmotorPower = ((turnerror * tkP) + (turnderivative * tkD) + (turntotalerror * tkI)) / 12.7; // TMP
   
         if (turnmotorPower > turnVoltagetotal) {
            turnmotorPower = turnVoltagetotal;
         } else if(turnmotorPower < (turnVoltagetotal * -1)) {
            turnmotorPower = (turnVoltagetotal * -1);
         }
   
         printf("TMP %f\n", turnmotorPower);
         Brain.Screen.setCursor(7, 1);
         Brain.Screen.print("TMP: ");
         Brain.Screen.print(turnmotorPower);
         // Rotational PID End
         
         rightMotors.spin(forward, (lateralmotorPower - turnmotorPower), volt);
         leftMotors.spin (forward, (lateralmotorPower + turnmotorPower), volt);
   
      } else {
         // Swing PID Start
         if (swingDirection == true) {

            float currentRotation = reduce_0_to_360(inertia14.rotation(degrees));

            swingerror = reduce_negative_180_to_180(desiredswingAngle - currentRotation);

            swingderivative = swingerror - swingpreverror;

            if (fabs(swingerror) < toleranceSwing) {
               swingtotalerror += swingerror;
            }

            swingpreverror = swingerror;

            if((currentRotation <= (desiredAngle + toleranceSwing)) && (currentRotation >= (desiredAngle - toleranceSwing))) {
               swingerror = 0;
               swingderivative = 0;
               swingtotalerror = 0;
               rightMotors.stop(coast);
               leftMotors.stop(coast);
               isSwinging = false;
               activePID = false;
            }

            printf("%f\n", currentRotation);

            swingmotorPower = ((swingerror * skP) + (swingderivative * skD) + (swingtotalerror * skI)) / 12.7;
            printf("SMP %f\n", swingmotorPower);

            if (swingmotorPower > swingVoltagetotal) {
               swingmotorPower = swingVoltagetotal;
            } else if(swingmotorPower < (swingVoltagetotal * -1)) {
               swingmotorPower = (swingVoltagetotal * -1);
            }

            leftMotors.spin(forward, swingmotorPower, volt);
            rightMotors.stop(hold);

         } else {

            float currentRotation = reduce_0_to_360(inertia14.rotation(degrees)*360.0/rotationScale);

            swingerror = reduce_negative_180_to_180(desiredswingAngle - currentRotation);

            swingderivative = swingerror - swingpreverror;

            if (fabs(swingerror) < swingI) {
               swingtotalerror += swingerror;
            }

            swingpreverror = swingerror;

            if((currentRotation <= (desiredAngle + toleranceSwing)) && (currentRotation >= (desiredAngle - toleranceSwing))) {
               swingerror = 0;
               swingderivative = 0;
               swingtotalerror = 0;
               rightMotors.stop(hold);
               leftMotors.stop(hold);
               isSwinging = false;
               activePID = false;
            }

            printf("CR %f\n", currentRotation);

            swingmotorPower = ((swingerror * skP) + (swingderivative * skD) + (swingtotalerror * skI)) / 12.7;

            if (swingmotorPower > swingVoltagetotal) {
               swingmotorPower = swingVoltagetotal;
            } else if(swingmotorPower < (swingVoltagetotal * -1)) {
               swingmotorPower = (swingVoltagetotal * -1);
            }

            printf("SMP %f\n", swingmotorPower);

            rightMotors.spin(reverse, swingmotorPower, volt);
            leftMotors.stop(hold);
         }
      }
      // Swing PID End
      task::sleep(50); // time between updates to the PID loop.
      Brain.Screen.clearScreen();
      cycles += 1;
      printf("Cycles: %i\n", cycles);
   }

   return(1);
}

// PID End

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

// distance should be in inches but if it is not you can always add a modifier angle is in degrees by default
void move(float distance, float angle) {
// convert linear distance to angular distance
   while (activePID == true) {
      // if another task is running do not run
      wait(5, msec);
   }
   if (activePID == false){

      leftMotors.setPosition(0, deg); // tell them they are at 0
      rightMotors.setPosition(0, deg); // tell them they are at 0
      // when PID is available for tasking, convert distance to degrees and/or tell it the desired angle to turn to and tell it that it is active
      float degreesWanted = (((distance*360)*gearRatio))/wheelCircumference;
      desiredDistance = degreesWanted;

      if(degreesWanted == 0) {
         turning = true;
      }

      desiredAngle = angle;
      activePID = true;
   }
}

// true for left false for right
void turn_to_angle(float angle, bool direction) {
   while (activePID == true) {
      // if another task is running do not run
      wait(5, msec);
   }
   if (activePID == false) {

      leftMotors.setPosition(0, deg); // tell them they are at 0
      rightMotors.setPosition(0, deg); // tell them they are at 0
      // when PID is available for tasking, tell it that it is swinging and that it is active
      // also tell it if it is left or right handed
      swingDirection = direction;
      desiredswingAngle = angle;
      isSwinging = true;
      activePID = true;
   }
}

int main() {
   prepSys();
   //enabledrivePID = true;
   //task PID( drivePID );
   //move(48.0, 0.0);
   desiredAngle = 90;
   turning = true;
   while(true){
         float currentHeading = reduce_0_to_360(inertia14.rotation(degrees)*360.0/rotationScale);
   
         printf("%f\n", currentHeading);
         Brain.Screen.setCursor(5, 1);
         Brain.Screen.print("CH: ");
         Brain.Screen.print(currentHeading);
   
         turnerror = reduce_negative_180_to_180(desiredAngle - currentHeading);
         
         // if both are true you are done moving if turn is true then you are done turning
         if((currentHeading <= (desiredAngle+toleranceTurn) && fabs(turnerror) >= (desiredAngle-toleranceTurn)) && (fabs(error) <= (toleranceLateral) && fabs(error) >= (-1*toleranceLateral))) {
            turnerror = 0;
            turnderivative = 0;
            turntotalerror = 0;
            error = 0;
            derivative = 0;
            totalerror = 0;
            
         } else if(currentHeading <= (desiredAngle+toleranceTurn) && fabs(turnerror) >= (desiredAngle-toleranceTurn)) {
            turnerror = 0;
            turnderivative = 0;
            turntotalerror = 0;
         } else if (currentHeading <= (desiredAngle+toleranceTurn) && fabs(turnerror) >= (desiredAngle-toleranceTurn) && turning == true) {
            turnerror = 0;
            turnderivative = 0;
            turntotalerror = 0;
            rightMotors.stop(coast);
            leftMotors.stop(coast);
         }
   
         turnderivative = turnerror - turnpreverror;
   
         if (currentHeading <= desiredAngle+turnI || currentHeading >= desiredAngle-turnI) {
            turntotalerror += turnerror;
         }
   
         turnpreverror = turnerror;
   
         float turnmotorPower = ((turnerror * tkP) + (turnderivative * tkD) + (turntotalerror * tkI)) / 12.7; // TMP
   
         if (turnmotorPower > turnVoltagetotal) {
            turnmotorPower = turnVoltagetotal;
         } else if(turnmotorPower < (turnVoltagetotal * -1)) {
            turnmotorPower = (turnVoltagetotal * -1);
         }
   
         printf("TMP %f\n", turnmotorPower);
         Brain.Screen.setCursor(7, 1);
         Brain.Screen.print("TMP: ");
         Brain.Screen.print(turnmotorPower);
         // Rotational PID End
         
         rightMotors.spin(forward, (-1*turnmotorPower), volt);
         leftMotors.spin (forward, (turnmotorPower), volt);
  }
   //wait(20, msec);
   //move(0, 90);
   //turn_to_angle(90, true);
   //enabledrivePID = false;
}

