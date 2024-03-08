#include "vex.h"
#include "pid.cpp"

using namespace vex;

class movement {

  float lkP = 0.4;
  float lkI = 0.004;
  float lkD = 0.5;
  
  float rkP = 0.4;
  float rkI = 0.004;
  float rkD = 0.5;
  
  float skP = 0.4;
  float skI = 0.004;
  float skD = 0.5;

  float settlebounds;
  
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
      movement(motor_group left, motor_group right, int inertialport, float gearratio, float wheeldiameter, float lkp, float lki, float lkd, float rkp, float rki, float rkd, float skp, float ski, float skd, int timeout, int settletime, float settlebounds, float aiwvalue) {
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
      }
  
      void move_distance(float distance, float desired_heading) {
        float degreesWanted = (((distance*360)*gearRatio))/circumference;
        float currentavgPosition = ((leftside.position(deg) + rightside.position(deg))/2);
        
        float heading = reduce_0_to_360(rotationalSensor.rotation());
        pid lateral = pid(lkP, lkI, lkD, 20, 1000, 100, 10);
      }
};