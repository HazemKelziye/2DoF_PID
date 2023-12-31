# 2 Degrees-of-Freedom PID Controller
This is an implementation of a PID controller to a landing rocket using Two action variables (Throttle & Thrusters)
these actions can be discrete or continuous.

I used the Rocket Lander Gym environment which is built-on top of the OpenAI Gym environment.
The PIDy controller was responsible for controlling the vertical and horizontal positioning of the rocket
using a setpoint of the following form { y = |x| - 1 }, by constructing this equation we can guarantee that the
rocket will be drived toward the center of the platform. The gain parameters for the PIDy controller are     
Kp = 15,000 , Ki = 10, Kd = 5,000. 
As for the other DoF which is the PIDtheta controller it maintained the rocket to be at equilibrium above the target
point i.e. (0,-1) and if it's not above that point it will tilt the rocket to make it move towards the target point,
the setpoint was {pi/4 * (x + Vx)}. The PIDtheta controller's gain parameters are Kp = 1,000 , Ki = 2.5, Kd = 750.<br>

## Landing demonstrations:<br>
<p align="center">
  <img src="figures/landing_rocket1.png" alt="Figure_1" width="550" height="700">
</p>
<p align="center">
  <img src="figures/landing_rocket3.png" alt="Figure_2" width="550" height="700">
</p>
<p align="center">
  <img src="figures/landing_rocket2.png" alt="Figure_3" width="550" height="700">
</p>

For clearer demonstrations please refer to this link => https://youtu.be/mLsDPvBsJDQ <br>


Here are the responses of the system, for multiple random landing samples;
<p align="center">
  <img src="figures/pid_f1.png" alt="Figure_4" width="600" height="400">
</p>
<p align="center">
  <img src="figures/pid_f3.png" alt="Figure_5" width="600" height="400">
</p>
<p align="center">
  <img src="figures/pid_f5.png" alt="Figure_6" width="600" height="400">
</p>

