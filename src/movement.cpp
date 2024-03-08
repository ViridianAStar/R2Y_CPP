#include "vex.h"
#include "pid.cpp"

using namespace vex;

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


  int port;
  float gearRatio;
  float circumference;
  motor_group leftside;
  motor_group rightside;
  inertial rotationalSensor = inertial(port);
  
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
      movement(motor_group left, motor_group right, int inertialport, float gearratio, float wheeldiameter, float lkp, float lki, float lkd, float rkp, float rki, float rkd, float skp, float ski, float skd, int timeout, int settletime, float TMV, float SMV, float LMV) {
        leftside = left;
        rightside = right;
        port = inertialport;
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

          float rotationalerror = reduce_negative_180_to_180(angle - heading);

          leftside.spin(forward, rotational.calcPID(rotationalerror), volt);
          rightside.spin(forward, -rotational.calcPID(rotationalerror), volt);

          task::sleep(10);
        }

        leftside.stop(hold);
        rightside.stop(hold);
      }
};