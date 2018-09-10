# Control of a 3D Quadrotor #

## The Goal of this Project ##

In the real world the flight controller is usually implemented in C or C++. So in this project you will implement your controller in C++. The code you write here can eventually be transferred to a real drone!

[image1]: ./result/1.gif
[image2]: ./result/2.gif
[image3]: ./result/3.gif
[image4]: ./result/4.gif
[image5]: ./result/5.gif
[image6]: ./result/6.gif


## 1. C++ Implementation ##

I used the CPPSim.pro of C++ in project folder in QtCreator IDE.
I implemented the functions to help to fly the drone in simulator.
and I made the setting parameters program in python with jupyter notebook.(modiParam folder)
from lots of test, finally I passed all scenarios.


## 2. Scenario Test ##
These parameters are tuned to optimize and to fly the drone.
- Mass
- kpPosXY
- kpPosZ
- kpVelXY
- kpVelZ
- kpBank
- kpYaw
- kpPQR.x,y,z


### Scenario 1 : Intro ###
![alt text][image1]

To pass this test, I tuned the Mass paramter to 0.5.

### Scenario 2 : AttitudeControl ###
![alt text][image2]
To pass this test, I tuned the kpPQR to [90, 90, 6] and kpBank to 8.
These paramters are major.

### Scenario 3 : PositionControl ###
![alt text][image3]
To pass this test, I tuned the 'kpYaw': to 2, kpVelXY to 12 and kpVelZ to 9.
Last two parameters could be chosed other values but this is essential to pass other test.

### Scenario 4 : Nonidealities ###
![alt text][image4]
I felt really hard to pass this scenario.
I focused the kpPosXY, kpPosZ and kiPosZ for tuning.
First time, Yellow colored fly is problem. It flied like snake. I tried almost 500 hundreds times to tune.
When I get the pass signal of Yellow one but I failed from Red one. and.. I got better values from both then middle one made trouble.
I found the bugs to my codes and fixed them and I got the pass.
I set the values kpPosXY to 30, kpPosZ to 20 and KiPosZ to 40.

### Scenario 5 : TrajectoryFollow###
![alt text][image5]

I found the red one's flight height is related to 'accelZCmd' of AltitudeControl.


### Scenario 6 : TestManyQuads ###
![alt text][image6]

Surprised! They are flying!
