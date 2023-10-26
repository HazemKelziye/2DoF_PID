# 2 Degrees-of-Freedom PID Controller
This is an implementation of a PID controller to a landing rocket using Two action variables (Throttle & Thrusters)
these actions can be discrete or continuous.

I used the Rocket Lander Gym environment which is built-on top of the OpenAI Gym environment.
The PIDy controller was responsible for controlling the vertical and horizontal positioning of the rocket
using a setpoint of the following form { y = |x| - 1 }. by constructing this equation we can guarantee that the
rocket will be drived toward the center of the platform.
As for the other DoF which is the PIDtheta controller it maintained the rocket to be at equilibrium above the target
point i.e. (0,-1) and if it's not above that point it will be tilt the rocket to make it move towards the target point,
the setpoint was {pi/4 * (x + Vx)}.

For more clear demonstrations please refer to this link => https://youtu.be/mLsDPvBsJDQ




https://github.com/Hazem-Kelziye/2DoF_PID/assets/147067179/6a48a94b-382b-4d63-b62d-6ab4ccf4b565


Here are the responses of the system, for multiple random landing samples;

![Figure_v3 7](https://github.com/Hazem-Kelziye/2DoF_PID/assets/147067179/ca840bd5-50fd-49e9-b632-f145a6a63d81)

![Figure_v3 5](https://github.com/Hazem-Kelziye/2DoF_PID/assets/147067179/ceac9090-c253-48b7-a9b9-de8a46b20994)

![Figure_v3 4](https://github.com/Hazem-Kelziye/2DoF_PID/assets/147067179/cb9bb53e-07ac-4bcc-9ac0-d52b829b6efd)
