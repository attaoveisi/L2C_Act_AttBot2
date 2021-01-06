# L2C_Act
This is the repository for the low level controlling of the AttBot2 which can be seen below:

![image](https://user-images.githubusercontent.com/17289954/103441769-704b2680-4c50-11eb-9d5e-821b64b49fd7.png)

![image](https://user-images.githubusercontent.com/17289954/103441772-7b9e5200-4c50-11eb-8961-9aa9ce3cf412.png)

![image](https://user-images.githubusercontent.com/17289954/103441777-848f2380-4c50-11eb-8dd9-7b64dbd9f221.png)

![image](https://user-images.githubusercontent.com/17289954/103441783-8eb12200-4c50-11eb-8d89-ec6a6a653c38.png)


## Structure, sensors, and actuators:

It consists of: 

  - an offroad chassis (borrowed from some cheap toy (30 EUR)) with two DC motors for two axis and an additional smaller DC motos for steering. 
  - two motor encoders (20 ppr) for estimating the velocity of front and rear axis.
  
  ![image](https://user-images.githubusercontent.com/17289954/103441873-578f4080-4c51-11eb-8650-754401ca683d.png)
  
  - two IMUs for observing (estimating) the front wheel angle 
  
  ![image](https://user-images.githubusercontent.com/17289954/103441925-ca98b700-4c51-11eb-9cae-ae7c9133cd23.png)
  
  - three ultrasound sensors for colllision avoidance (and a buzzer to let me know if a collision may happen), two in front and one in back

## 

In order to be able to drive the robot, two methods are implemented:

1) The robot movement can be controlled by a remote controller (manual drive) where I have used a `KY-022 Infrared Sensor Receiver Module` together with a cheap remote controller. 

2) The robot can be controlled from the virtual driver (path and motion planner) from the ROS world. For this purposes, a bangbang controller is implemented which can allow the robot to move forward and backword as well as steer in arbitrary direction. 

The latter is achieved by using rosserial package (rosserial_python) where I have written a costum ROS messaege to perform these. The interface is relaized by a raspberry Pi.
