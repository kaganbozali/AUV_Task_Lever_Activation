# AUV_Task_Lever_Activation

The aim of the study is approaching the lever while adjusting the lateral and vertical bosition at the same time using controller.
**Introduction**

The aim of this study is to detect the lever in the simulation environment of the AUV with the help
of a camera and to separate the colors in the lever and to activate the lever that approaches the lever
according to the user information by using the separated colors. While the AUV approaching the
lever, it also has the adjust lateral position and depth according to position of levers.
**1. Methodology**

**1.1 Proportional Control**

The necessary processes to write a simple proportional control were first continued by calculating
the error value and calculating the P function using the calculated error value according to Eq-1,2.
Error=Dobtained −D reel                                                                         (Eq-1)
P=K p∗Error                                                                                     (Eq-2)
After the calculation of the P function, the proportional controller ensures that the vehicle reaches
the desired value when needed, and it returns when the desired value is exceeded, and it does this in
an infinite loop.
**1.2 Colour Detection**

Detection of colour is defined in range for this simulation. To explain it simply, an equation in the
form of a first-order Eq-3 calculates the initial condition and the condition it reaches, and provides
the calculation of coefficients A and B. The y value given in this equation is the speed and the x
value is the radius value of the colour values read.
Y = A+ BX                                                                                      (Eq-3)
After the initial and final calculations are made, the values in between are calculated at each step
during the simulation and the vehicle is allowed to move. Also it has to be mentioned that X values
are mean values of the lever’s radius.
**2. Simulation**
According to chosen strategy, first AUV has to ensure its lateral using position according to lever
position using PI controller, then same like lateral position it has to adjust its depth also with the
help of the controller. Then AUV will reach the somewhere around the lever using colour detection.
It basically detect the lever and calculate the radius of the user desired colour. Lastly, it grabs the
lever and activates with two different actions.
**2.1 Adjusting Lateral Position and Depth**
Adjusting lateral position and depth has performed with using controllers. Practically, lateral
position adjustment performed first and then depth adjustment comes. These two processes happens
one by one however both of the controllers are operating until the end of the simulation. For
example initial and last position of the AUV has shown in Figure-1.

![image](https://github.com/kaganbozali/AUV_Task_Lever_Activation/assets/104154215/68e94dea-9987-4469-ae3f-8b1f2e8214e1)
**Figure 1.** Depth Adjustment

and the output of the depth controller is given in the Figure 2.

![image](https://github.com/kaganbozali/AUV_Task_Lever_Activation/assets/104154215/69de10ac-7a04-46e9-bde0-72a9a6a2c40e)
**Figure 2.** Output

**2.3 Approaching**
Approaching performs according the radius values of the lever. There are one lower and one upper
limit for radius and speed values which lower radius value corresponds to maximum speed while
higher corresponds to minimum. Simulation is calculating that speed values every time inside the
loop and reading the corresponding speed values. It moves until it reaches the lever however there
is no sensor that express that vehicle reached to destination so determination of the limits has
crucial role.

![image](https://github.com/kaganbozali/AUV_Task_Lever_Activation/assets/104154215/6b4c649a-35f7-480a-ab71-a0948fd59144)
**Figure 3.** Camera View of AUV in Approaching Task

![image](https://github.com/kaganbozali/AUV_Task_Lever_Activation/assets/104154215/f4ef6364-6997-4e49-bd7d-d8fd177829df)
**Figure 4.** Output of the Approach Task

**2.4 Grabbing and Activation of Lever**
Grabbing and activation is relatively harder task to perform when we compare to other tasks. The
reason for that approaching part performs using colour detection and detecting the radius values of
the colours could vary with lots of factors. However simulation is capable to do it as you see
according to Figure 5. When the AUV reached the exact position first it grabs the circle in front of
the lever and then, it closes the grabber and first it pushes to lever then pulls it. In the figures below
activation of the green lever has shown.

![image](https://github.com/kaganbozali/AUV_Task_Lever_Activation/assets/104154215/4637f37e-207e-444f-83d6-037f82fcdaa1)
**Figure 5.** Grabbing and Activation of Lever

**Conclusion**

When we investigate the results it can be say that, some operations are hard to perform in
underwater and you need some additional sensor to finish simulation. For example, after the
submerging process, AUV totally lost its connection with GPS and result of this robot is totally
blind underwater. In this study AUV find its way using colour detection however it does not always
come with consistent results and it could effect the grabbing part.
There are some issues that it is hard to perform without sensor and some of the problems that can be
easily solved with sensors which:
• The problem that arises from the method used to approach for grabbing can be easily solved
with a distance sensor.
• In the case where we have stated that the grabbing task is the most complex task, an extra
alarm in the grabber will always appear when it touches the lever, which will facilitate this
part.
• Also when we use different controllers for different degrees of freedom it can causes
unwanted motions. So in order to solve this problem, more complex controller has to use to
avoid unwanted motions.
To reach a conclusion in general, the definitions given in the task were concluded successfully,
albeit in a fragmented way.
