#include "vex.h"
#include "pid.cpp"

using namespace vex;
  
  // Functions borrowed from JAR-Template Start
  float movement::reduce_0_to_360(float angle) {
    while(!(angle >= 0 && angle < 360)) {
      if( angle < 0 ) { angle += 360; }
      if(angle >= 360) { angle -= 360; }
    }
    return(angle);
  }
  
  float movement::reduce_negative_180_to_180(float angle) {
    while(!(angle >= -180 && angle < 180)) {
      if( angle < -180 ) { angle += 360; }
      if(angle >= 180) { angle -= 360; }
    }
    return(angle);
  }
  
  float movement::reduce_negative_90_to_90(float angle) {
    while(!(angle >= -90 && angle < 90)) {
      if( angle < -90 ) { angle += 180; }
      if(angle >= 90) { angle -= 180; }
    }
    return(angle);
  }
  // Functions borrowed from JAR-Template End
  
    movement::movement(motor_group left, motor_group right, float gearratio, float wheeldiameter, float lkp, float lki, float lkd, float rkp, float rki, float rkd, float skp, float ski, float skd, int timeout, int settletime, float TMV, float SMV, float LMV, float LaiwValue, float RaiwValue, float SaiwValue, float LsettleBounds, float RsettleBounds, float SsettleBounds, float wheelBase) :
        leftside(left),
        rightside(right),
        lkP(lkp),
        lkI(lki),
        lkD(lkd),
        rkP(rkp),
        rkI(rki),
        rkD(rkd),
        skP(skp),
        skI(ski),
        skD(skd),
        settleTime(settletime),
        Timeout(timeout),
        tmv(TMV),
        lmv(LMV),
        smv(SMV),
        gearRatio(gearratio),
        circumference(M_PI * wheeldiameter),
        laiwValue(LaiwValue),
        raiwValue(RaiwValue),
        saiwValue(SaiwValue),
        lsettleBounds(LsettleBounds),
        rsettleBounds(RsettleBounds),
        ssettleBounds(SsettleBounds),
        WheelBase(wheelBase)
      {};

      // go a distance
      void movement::move_distance(float distance) {

        // first we convert desired linear distance to degrees
        float degreesWanted = ((distance*360)/(circumference*gearRatio));
        printf("desired %f\n", degreesWanted);
        // we record the initial average position of the motors
        float initialavgPosition = ((leftside.position(deg) + rightside.position(deg))/2);
        printf("init %f\n", initialavgPosition);
        // your start average position is your initial average position
        float avgPositon = initialavgPosition;

        // record your initial heading
        float heading = reduce_0_to_360(rotationalSensor.rotation());
        // your desired heading is your initial heading this keeps you on a straight line
        float desired_heading = heading;

        // initialize PIDs
        pid lateral = pid(lkP, lkI, lkD, laiwValue, Timeout, settleTime, lsettleBounds, lmv);
        pid rotational = pid(rkP, rkI, rkD, raiwValue, Timeout, settleTime, rsettleBounds, tmv);
        
        while (lateral.active() == true) {
          // while youre lateral pid is active calculate your average position and heading
          avgPositon = ((leftside.position(deg) + rightside.position(deg))/2);
          heading = reduce_0_to_360(rotationalSensor.rotation());

          // calculate your errors to give to PID (we say that our desired position is our initial average positon + our desired distance)
          float lateralerror = (degreesWanted + initialavgPosition) - avgPositon;
          printf("lerror %f\n", lateralerror);
          float headingerror = reduce_negative_180_to_180(desired_heading - heading);

          // spin our motors with the desired amounts of power.
          leftside.spin(forward, (lateral.calcPID(lateralerror) + rotational.calcPID(headingerror)), volt);
          rightside.spin(forward, (lateral.calcPID(lateralerror) - rotational.calcPID(headingerror)), volt);

          // stop doing things for 10 milliseconds after every loop
          task::sleep(10);
        }

        // when its not active, stop.
        leftside.stop(hold);
        rightside.stop(hold);
      }

      // go a distance while turning
      void movement::move_distance_spiral(float distance, float desired_end_angle) {

        // first we convert desired linear distance to degrees
        float degreesWanted = ((distance*360)/(circumference*gearRatio));
        printf("desired %f\n", degreesWanted);
        // we record the initial average position of the motors
        float initialavgPosition = ((leftside.position(deg) + rightside.position(deg))/2);
        printf("init %f\n", initialavgPosition);
        // your start average position is your initial average position
        float avgPositon = initialavgPosition;

        // record your initial heading
        float heading = reduce_0_to_360(rotationalSensor.rotation());
        // your desired heading is your initial heading this keeps you on a straight line
        float desired_heading = heading;

        // initialize PIDs
        pid lateral = pid(lkP, lkI, lkD, laiwValue, Timeout, settleTime, lsettleBounds, lmv);
        pid rotational = pid(rkP, rkI, rkD, raiwValue, Timeout, settleTime, rsettleBounds, (lmv*0.6));
        
        while ((lateral.active() == true && rotational.active() == true) || (lateral.active() == false && rotational.active() == true)) {
          // while youre lateral pid is active calculate your average position and heading
          avgPositon = ((leftside.position(deg) + rightside.position(deg))/2);
          heading = reduce_0_to_360(rotationalSensor.rotation());

          // calculate your errors to give to PID (we say that our desired position is our initial average positon + our desired distance)
          float lateralerror = (degreesWanted + initialavgPosition) - avgPositon;
          printf("lerror %f\n", lateralerror);
          float headingerror = reduce_negative_180_to_180(desired_heading - heading);

          // spin our motors with the desired amounts of power.
          leftside.spin(forward, (lateral.calcPID(lateralerror) + rotational.calcPID(headingerror)), volt);
          rightside.spin(forward, (lateral.calcPID(lateralerror) - rotational.calcPID(headingerror)), volt);

          // stop doing things for 10 milliseconds after every loop
          task::sleep(10);
        }

        // when its not active, stop.
        leftside.stop(hold);
        rightside.stop(hold);
      }

      // point front towards a given angle.
      void movement::point_at_angle(float angle) {
        // first we get our intial heading
        float heading = reduce_0_to_360(rotationalSensor.rotation());

        // we initialize our rotational PID.
        pid rotational = pid(rkP, rkI, rkD, raiwValue, Timeout, settleTime, rsettleBounds, tmv);

        // while we aren't within the accepted values for our desired angle
        while (rotational.active() == true) {
          // our current heading
          heading = reduce_0_to_360(rotationalSensor.rotation());

          // how far we are from our desired heading
          float rotationalerror = reduce_negative_180_to_180(angle - heading);

          // rotate both sets of motors in opposite directions.
          leftside.spin(forward, rotational.calcPID(rotationalerror), volt);
          rightside.spin(forward, -rotational.calcPID(rotationalerror), volt);

          // do nothing for 10 milliseconds
          task::sleep(10);
        }

        // when we are complete stop.
        leftside.stop(hold);
        rightside.stop(hold);
      }

      // swing towards an angle using the left side
      void movement::swing_towards_angle_left(float angle) {
        // get your initial heading
        float heading = reduce_0_to_360(rotationalSensor.rotation());

        // for the side that is moving, get your intial position to reset too. This is for linear accuracy if you move forwards later.
        float retpos = leftside.position(deg);

        // initialize your swing PID
        pid swing = pid(skP, skI, skD, saiwValue, Timeout, settleTime, ssettleBounds, smv);

        // while it is active:
        while (swing.active() == true) {
          
          // get your current heading
          heading = reduce_0_to_360(rotationalSensor.rotation());

          // calculate your error
          float swingerror = reduce_negative_180_to_180(angle - heading);

          // calculate your power
          float power = swing.calcPID(swingerror);

          // move ONLY THE DESIRED SIDE towards that angle.
          leftside.spin(forward, power, volt);

          // stop the non-desired side
          rightside.stop(hold);

          // do nothing for 10 milliseconds
          task::sleep(10);
        }
        
        // when complete stop all motors and return the side that moved to its original encoder value.
        leftside.stop(hold);
        rightside.stop(hold);
        leftside.setPosition(retpos, deg);
      }

      // swing towards an angle using the right side
      void movement::swing_towards_angle_right(float angle) {
        float heading = reduce_0_to_360(rotationalSensor.rotation());

        float retpos = rightside.position(deg);

        pid swing = pid(skP, skI, skD, saiwValue, Timeout, settleTime, ssettleBounds, smv);
        while (swing.active() == true) {

          heading = reduce_0_to_360(rotationalSensor.rotation());
          float swingerror = reduce_negative_180_to_180(angle - heading);

          float power = swing.calcPID(swingerror);

          rightside.spin(forward, -power, volt);

          leftside.stop(hold);

          task::sleep(10);
        }
        
        leftside.stop(hold);
        rightside.stop(hold);
        rightside.setPosition(retpos, deg);
      }

      void movement::set_lateral_tuning(float p, float i, float d) {
        lkP = p;
        lkI = i;
        lkD = d;
      }

      void movement::set_rotational_tuning(float p, float i, float d) {
        rkP = p;
        rkI = i;
        rkD = d;
      }
      
      void movement::set_swing_tuning(float p, float i, float d) {
        skP = p;
        skI = i;
        skD = d;
      }
      
      void movement::set_timeout(int t) {
        Timeout = t;
      }
      
      void movement::set_settle_time(int t) {
        settleTime = t;
      }
      
      void movement::set_voltage_limits(float LMV, float TMV, float SMV) {
        lmv = LMV;
        tmv = TMV;
        smv = SMV;
      }

movement::tankDrive::tankDrive(motor_group left, motor_group right) : 
  TD_leftside(left),
  TD_rightside(right)
{};

void movement::tankDrive::right_handed_userDrive() {
  TD_leftside.spin(forward, ((driver.Axis2.value() + driver.Axis4.value())/10), volt);
  TD_rightside.spin(forward, ((driver.Axis2.value() - driver.Axis4.value())/10), volt);
}

void movement::tankDrive::single_stick_right_handed_userDrive() {
  TD_leftside.spin(forward, ((driver.Axis2.value() + driver.Axis1.value())/10), volt);
  TD_rightside.spin(forward, ((driver.Axis2.value() - driver.Axis1.value())/10), volt);
}

void movement::tankDrive::left_handed_userDrive() {
  TD_leftside.spin(forward, ((driver.Axis3.value() + driver.Axis1.value())/10), volt);
  TD_rightside.spin(forward, ((driver.Axis3.value() - driver.Axis1.value())/10), volt);
}

void movement::tankDrive::single_stick_left_handed_userDrive() {
  TD_leftside.spin(forward, ((driver.Axis3.value() + driver.Axis4.value())/10), volt);
  TD_rightside.spin(forward, ((driver.Axis3.value() - driver.Axis4.value())/10), volt);
}