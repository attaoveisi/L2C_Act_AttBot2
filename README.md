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
  
  
  - three ultrasound sensors for colllision avoidance (and a buzzer to let me know if a collision may happen)
