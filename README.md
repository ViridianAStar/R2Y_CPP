# R2Y_CPP PID Motion Program

## Background

You may wish to tune specific values for your system. 3 functions for reducing headings where borrowed from the [JAR-Template](https://github.com/JacksonAreaRobotics/JAR-Template). This was made for VEX V5 system using Microsoft Visual Studio Code and the VEX V5 extension.


## Overview of PID

> Q: What is PID?

> A: PID is a system that accounts for your error or distance from a desired heading or position.

Here is the actual code for PID used in the program: 

```cpp
        // calculate pid voltage
        float calcPID(float error) {
            
            // first we check if our error is less than your anti integral windup value, if it is
            // we start compounding.
            if (fabs(error) < aiwValue) {
                integral += error;
            }

            // if your error crosses 0 eliminate your integral
            if ((error < 0 && preverror > 0) || (error > 0 && preverror < 0)) {
                integral = 0;
            }

            // your derivitave is how far away you were last cycle minus how far away you are this cycle.
            derivative = preverror - error;

            // to calculate our raw power/voltage value we multiply our error derivative and integral values by their respective
            // tuning values then divide by 12.7 as we are working with motors that go up to 12.7 volts.
            float rawvalue = ((kP*error) + (kI * integral) + (kD * derivative)) / 12.7;
            
            // set our previous error for our next run
            preverror = error;

            // check to see if we are withing settle bounds if yes add time if no set time to 0
            if (fabs(error) < settleBounds) {
                settledtime += 10;
            } else {
                settledtime = 0;
            }

            // add running time for timeout
            runningtime += 10;
            
            // return the output ensuring its value is within your voltage limits
            return calculateoutput(rawvalue);
        }
```

> Q: how does the PID know when it is done?

> A: It detects when it is within bounds using built in motor encoders and the inertial sensors rotation. When it is within bounds it waits for a while to make sure it is staying within bounds.

Bounds Detection Code: 

```cpp
        bool active() {
            // if your timeout is reached and not 0 as 0 would be infinite you are complete
            if (Timeout != 0 && runningtime >= Timeout) {
                return false;
            }
            
            // if your settled time is equal to or greater than your settle time you are complete
            if (settledtime >= settleTime) {
                return false;
            }

            // otherwise you are not.
            return true;
        }
```

these are the 2 main and basic functions of the PID system.

## Overview of Movement / Motion

> Q: Is this odometery?

> A: No, however this does allow the robot to move specified distances and rotate to specified angles.

> Q: Are there plans for odometery in the future?

> A: Yes! There is a plan to encorporate odometery in the future.

> Linear Motion or move_distance function:

```cpp
      // go a distance
      void movement::move_distance(float distance) {

        // first we convert desired linear distance to degrees
        float degreesWanted = ((distance*360)/(circumference*gearRatio));
        // we record the initial average position of the motors
        float initialavgPosition = ((leftside.position(deg) + rightside.position(deg))/2);

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
```

**Note**: This function combines two different types of PID 1 to control its heading and the other to control its linear speed. This allows it to move with smoothing and move in a straight line. This also means it can correct for variances in its motors and path over time. This is the only function that currently employs 2 different types of PID to perform 1 action.

> Rotational control or point_at_angle function:

```cpp
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
```

**Note**: This only uses 1 PID and rotates both motors. This is different than a swing which only rotates one side. 

> Swing control or swing_towards_angle_side function:

```cpp
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
```

**Note**: The same function is used for the right side however the group that is moving is switched.

## Other
If you find a bug please open an issue on GitHub and we will try and fix it in a timely manner.
This is licensed under an MIT license