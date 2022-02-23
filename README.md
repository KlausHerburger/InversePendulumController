# InversePendulumController
In this project a controller for an inverse pendulum in the unstable standing position is designed. Flatness-based feedforward control allows the pendulum to be laterally displaced for arbitrary carriage positions. The control concepts are checked and validated in Simulink.

![ezgif com-gif-maker (2)](https://user-images.githubusercontent.com/63397065/155312829-f2fcd38a-8161-417e-b6b7-697f0cccc080.gif)

## Concept

In the first step, the mathematical model of the pendulum system was derived and linearized for the subsequent controller design. 
A velocity controller was designed for the carriage without pendulum attachment, which could be implemented sufficiently fast and robust using a PI controller with anti-windup. This was followed by the design of a controller to stabilize the inverse pendulum at the upper rest position. For this purpose, an optimal Ricatti controller was calculated, which minimizes a quality measure adapted to the state model. The states are thereby estimated by a Luenberger observer. 
Subsequently, the stabilizing controller was extended by a flatness-based feedforward control, which allows lateral displacement of the pendulum for arbitrary carriage positions in a given time. For this purpose, a polynomial approach was used to generate the desired trajectory for lateral displacement and, using a previously defined flat output, to compute the resulting state trajectories and setpoint trajectories. In the two-degree-of-freedom controller structure, the setpoint trajectories are used for feedforward control and provide good guidance behavior. The calculated state trajectories are fed into the Ricatti controller together with the observed states so that additional disturbances can be suppressed. 

## Results

Carriage position and pendulum angle for the carriage starting position <img src="https://latex.codecogs.com/svg.image?w_0&space;=&space;0m" title="w_0 = 0m" />, the carriage desired final position <img src="https://latex.codecogs.com/svg.image?w_f&space;=&space;0.2m" title="w_f = 0.2m" /> and the transition time <img src="https://latex.codecogs.com/svg.image?T&space;=&space;1.5s&space;" title="T = 1.5s " />. 

Without disruptions:

![values](https://user-images.githubusercontent.com/63397065/155294025-e5b187b3-d2a5-4dc3-86dd-6c7112b25d53.jpg)

With disruptions:

![values3](https://user-images.githubusercontent.com/63397065/155294091-f2232e87-e8eb-4468-bf2e-eec87f295f51.jpg)
