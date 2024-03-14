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
  
    movement::movement(motor_group left, motor_group right, float gearratio, float wheeldiameter, float lkp, float lki, float lkd, float rkp, float rki, float rkd, float skp, float ski, float skd, int timeout, int settletime, float TMV, float SMV, float LMV, float LaiwValue, float RaiwValue, float SaiwValue, float LsettleBounds, float RsettleBounds, float SsettleBounds) :
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
        ssettleBounds(SsettleBounds)
      {};
  
      void movement::move_distance(float distance) {
        float degreesWanted = (((distance*360)*gearRatio))/circumference;
        float initialavgPosition = ((leftside.position(deg) + rightside.position(deg))/2);
        float avgPositon = initialavgPosition;
        float heading = reduce_0_to_360(rotationalSensor.rotation());
        float desired_heading = heading;
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

      void movement::point_at_angle(float angle) {
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

      void movement::swing_towards_angle_left(float angle) {
        float heading = reduce_0_to_360(rotationalSensor.rotation());

        float retpos = leftside.position(deg);

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
        leftside.setPosition(retpos, deg);
      }

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