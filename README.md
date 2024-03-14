# R2Y_CPP PID Motion Program

## Background

You may wish to tune specific values for your system. 3 functions for reducing headings where borrowed from the [JAR-Template](https://github.com/JacksonAreaRobotics/JAR-Template)


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