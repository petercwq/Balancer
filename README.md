# Balancer

## Stepper Motor Control

[Control of Stepping Motors](http://homepage.divms.uiowa.edu/~jones/step/)

"You close the loop on position. The farther the motor is from the target, the bigger the error. The bigger the error, the bigger the velocity target. The bigger the velocity target, the more steps per second you command towards the position target."

[Matlab Stepper Motor Model](https://cn.mathworks.com/help/physmod/sps/powersys/ref/steppermotor.html?requestedDomain=true)
```matlab
    power_steppermotor
```
### No PID Controller

"You do not need to do PID with steppers. All you do is send the stepper driver a pulsestream to follow and set a direction bit. NO pid is used. Feedback with steppers is typically used only to confirm commanded position is reached and to determine how much correction may be required. But correction is done after the move is
completed."

"If your robot arm doesn't move fast, there is no need to use whole PID. This situation takes place when time to cross a deadband of position with
full speed is longer then summarized times of ac- and deceleration. It seems to be enough using a simple proportional controller (only P term).
But you have to implement switching frequency limit which cannot be exceeded. That's the simplest way.

On the other hand, if you require very fast movements and accuracy PID should be implemented."

### Speed Control - PI controller


### Position Control - PID controller

