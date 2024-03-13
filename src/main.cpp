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

class pid {
    int Timeout;
        
    int settleTime;
            
    float integral;
           
    float derivative;
          
    float preverror;

    // p tuning constant
    float kP;

    // i tuning constant
    float kI;

    // d tuning constant
    float kD;

    // value at which integral value starts compounding
    float aiwValue;
    
    float settleBounds;

    int runningtime = 0;

    int settledtime = 0;

    // maximum voltage
    float max = 12.7;

    public:
        pid(float kp, float ki, float kd, float aiwvalue, int timeout, int settletime, float settlebounds, float Max){
           kP = kp;
           kI = ki;
           kD = kd;
           aiwValue = aiwvalue;

           if ((timeout % 10) >= 5) {
               Timeout = timeout + (10 - (timeout % 10));
           } else if ((timeout % 10) >= 1) {
               Timeout = timeout - (timeout % 10);
           } else {
             Timeout = timeout;   
           }

           if ((settletime % 10) >= 5) {
               settleTime = settletime + (10 - (settletime % 10));
           } else if ((settletime % 10) >= 1) {
               settleTime = settletime - (settletime % 10);
           } else {
             settleTime = settletime;   
           }

           settleBounds = settlebounds;

           max = Max;
        }
        
        // ensure voltage mins and maxes on inputVoltage
        float calculateoutput(float inputVoltage) {

            if (inputVoltage < -max) {
                return -max;
            } else if (inputVoltage > max) {
                return max;
            }

            return inputVoltage;
        }

        // calculate pid voltage
        float calcPID(float error) {
            
            if (fabs(error) < aiwValue) {
                integral += error;
            }

            if ((error < 0 && preverror > 0) || (error > 0 && preverror < 0)) {
                integral = 0;
            }

            derivative = preverror - error;

            float rawvalue = ((kP*error) + (kI * integral) + (kD * derivative)) / 12.7;

            preverror = error;

            if (fabs(error) < settleBounds) {
                settledtime += 10;
            } else {
                settledtime = 0;
            }

            runningtime += 10;
            
            return calculateoutput(rawvalue);
        }

        bool active() {
            if (Timeout != 0 && runningtime >= Timeout) {
                return false;
            }

            if (settledtime >= settleTime) {
                return false;
            }

            return true;
        }
};  


class movement {

  float lkP;
  float lkI;
  float lkD;
        
  float rkP;
  float rkI;
  float rkD;
        
  float skP;
  float skI;
  float skD;

  float lsettleBounds = 5;
  float rsettleBounds = 5;
  float ssettleBounds = 5;
  int settleTime;
  int Timeout;

  float laiwValue = 20;
  float raiwValue = 15;
  float saiwValue = 15;

  float tmv;
  float smv;
  float lmv;

  vex::inertial rotationalSensor = vex::inertial( vex::PORT14 );
  float gearRatio;
  float circumference;
  motor_group leftside;
  motor_group rightside;
  
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
  
  
  public:
      movement(motor_group left, motor_group right, float gearratio, float wheeldiameter, float lkp, float lki, float lkd, float rkp, float rki, float rkd, float skp, float ski, float skd, int timeout, int settletime, float TMV, float SMV, float LMV) {
        leftside = left;
        rightside = right;
        circumference = M_PI * wheeldiameter;
        gearRatio = gearratio;

        lkP = lkp;
        lkI = lki;
        lkD = lkd;

        rkP = rkp;
        rkI = rki;
        rkD = rkd;

        skP = skp;
        skI = ski;
        skD = skd;

        settleTime = settletime;
        Timeout = timeout;

        tmv = TMV;
        lmv = LMV;
        smv = SMV;
      }
  
      void move_distance(float distance, float desired_heading) {
        float degreesWanted = (((distance*360)*gearRatio))/circumference;
        float initialavgPosition = ((leftside.position(deg) + rightside.position(deg))/2);
        float avgPositon = initialavgPosition;
        float heading = reduce_0_to_360(rotationalSensor.rotation());
        pid lateral = pid(lkP, lkI, lkD, laiwValue, Timeout, settleTime, lsettleBounds, lmv);
        pid rotational = pid(rkP, rkI, rkD, raiwValue, Timeout, settleTime, rsettleBounds, tmv);
        
        while (lateral.active() == true) {
          avgPositon = ((leftside.position(deg) + rightside.position(deg))/2);
          heading = reduce_0_to_360(rotationalSensor.rotation());
          float lateralerror = (degreesWanted + initialavgPosition) - avgPositon;
          float headingerror = reduce_negative_180_to_180(desired_heading - heading);

          leftside.spin(forward, (lateral.calcPID(lateralerror) + rotational.calcPID(headingerror)), volt);
          rightside.spin(forward, (lateral.calcPID(lateralerror) - rotational.calcPID(headingerror)), volt);

          task::sleep(10);
        }

        leftside.stop(hold);
        rightside.stop(hold);
      }

      void point_at_angle(float angle) {
        float heading = reduce_0_to_360(rotationalSensor.rotation());

        pid rotational = pid(rkP, rkI, rkD, raiwValue, Timeout, settleTime, rsettleBounds, tmv);

        while (rotational.active() == true) {
          heading = reduce_0_to_360(rotationalSensor.rotation());
          float rotationalerror = reduce_negative_180_to_180(angle - heading);

          leftside.spin(forward, rotational.calcPID(rotationalerror), volt);
          rightside.spin(forward, -rotational.calcPID(rotationalerror), volt);

          task::sleep(10);
        }

        leftside.stop(hold);
        rightside.stop(hold);
      }

      void swing_towards_angle_left(float angle) {
        float heading = reduce_0_to_360(rotationalSensor.rotation());

        pid swing = pid(skP, skI, skD, saiwValue, Timeout, settleTime, ssettleBounds, smv);
        while (swing.active() == true) {

          heading = reduce_0_to_360(rotationalSensor.rotation());
          float swingerror = reduce_negative_180_to_180(angle - heading);

          float power = swing.calcPID(swingerror);

          leftside.spin(forward, power, volt);

          rightside.stop(hold);

          task::sleep(10);
        }

        leftside.stop(hold);
        rightside.stop(hold);
      }

       void swing_towards_angle_right(float angle) {
        float heading = reduce_0_to_360(/*rotationalSensor.rotation()*/ 10);

        pid swing = pid(skP, skI, skD, saiwValue, Timeout, settleTime, ssettleBounds, smv);
        while (swing.active() == true) {
        heading = reduce_0_to_360(rotationalSensor.rotation());
          float swingerror = reduce_negative_180_to_180(angle - heading);

          float power = swing.calcPID(swingerror);

          rightside.spin(forward, power, volt);

          leftside.stop(hold);

          task::sleep(10);
        }
        
        leftside.stop(hold);
        rightside.stop(hold);
      }
};

//movement driveControl = movement(leftMotors, rightMotors, gearRatio, float(3.25), float(.5), float(0.005), float(0.7), float(0.8), float(0.008), float(0.5), float(0.75), float(0.0075), float(0.5), 1000, 150, float(10), float(9), float(10.7));

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

void printvalues(int tval) {
   Brain.Screen.setCursor(5, 7);
   Brain.Screen.print(inertia14.rotation());
   wait(tval, msec);
   Brain.Screen.clearScreen();
}

void square() {
   movement driveControl = movement(leftMotors, rightMotors, gearRatio, float(3.25), float(.5), float(0.005), float(0.7), float(0.8), float(0.008), float(0.5), float(3.8), float(0.055), float(.75), 1000, 150, float(10), float(9), float(10.7));
   printvalues(500);
   driveControl.move_distance(100, 0);
   printvalues(500);
   driveControl.swing_towards_angle_right(90);
   printvalues(500);
   driveControl.move_distance(100, 90);
   printvalues(500);
   driveControl.swing_towards_angle_right(180);
   printvalues(500);
   driveControl.move_distance(100, 180);
   printvalues(500);
   driveControl.swing_towards_angle_right(-90);
   printvalues(500);
   driveControl.move_distance(100, -90);
   printvalues(500);
   driveControl.swing_towards_angle_right(0);
   printvalues(500);
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
   //movement driveControl = movement(leftMotors, rightMotors, gearRatio, float(3.25), float(.5), float(0.005), float(0.7), float(0.8), float(0.008), float(0.5), float(3.8), float(0.055), float(.75), 1000, 150, float(10), float(9), float(10.7));
   //driveControl.move_distance(500, 0);
   //driveControl.swing_towards_angle_left(90);
   /*while (1) {
      userDrive();
   }*/
   
}

