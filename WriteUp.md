# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

### Proportional Parameter

The proportional parameter effects the control output directly based on the current error from the desired position. P part imparts a overshoot on the system and makes the system osciallte.

### Derivative Parameter

Derivative part acts upon the difference of the current and previous error. It smoothens the trajectory of the system and but slows down the system (time to reach the goal is delayed). But the PD controller imparts a bias meaning it never reaches the actual goal.

### Integral Parameter

The integral parameter acts on the sum of errors accumulated in a time period. It helps in eliminating the bias of the system.


### Final parameters

the parameters were initially selected by trial and error with P = 0.1 and D = 1.5 and I  = 0.001. These values are used for initializing twiddle. Twiddle then is used to fine tune the selected P,I and values.

Output PID values from the twiddle are used to run the simulation. the video link can be found [here.](https://www.youtube.com/watch?v=P24BwquN420&feature=youtu.be)