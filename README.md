# This is the Arduino MRSC (Micro Rc Stability Control) for WLtoys 5 pin servos!
## Features:
- For WLtoys A959, A969, A979 RC cars
- Provides about the same functionality as the Traxxas TSM stability management system
- The car tries to follow the direction, which is requested from the steering wheel. Deviations are compensated with countersteering
- I also have an Adruino RC system with fully integrated MRSC: https://github.com/TheDIYGuy999/Micro_RC_Receiver
- Version for 3 pin standard RC servos: https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo
- Programmable with Arduino IDE
- Input for feedback potentiometer of a WLtoys 5 pin servo
- The simulated potentimeter feedback is sent to the WLtoys receiver / ESC
- Uses a standard Sparkfun "Pro Micro" 8MHz, 3.3V or an 8MHz, 3.3V Pro Mini
- MPU-6050 gyro / accelerometer wired to the I2C pins
- Gain potentiometer
- Input for gyro direction inversion (if the MPU-6050 is mounted upside down)

## Changelog:

New in V 0.1
- Initial commit
- Not yet tested in a car

New in V 0.2
- Built a compact sandwich with MPU-6050 and Pro Micro Board, including a low pass filter for the analog output
- Software cleaned up and optimized. Gyro calibration removed. Board is now immedialtely ready after power on.
- First indoor test in a WLtoys A979-B done
- Both the pot and the direction switches are disabled. So the standard values Gain 20% & Inversed = true are active. You can activate them in the main loop, if required.

## Usage

See videos and pictures
- Build video: https://www.youtube.com/watch?v=pNb_3yC4mTQ&t=8s
- Outdoor test video: https://www.youtube.com/watch?v=cW0dXprR9rY&t=1s
- More informations  on my YouTube channel: https://www.youtube.com/channel/UCqWO3PNCSjHmYiACDMLr23w


Vehicle board top side
![](https://github.com/TheDIYGuy999/MRSC_Adapter_WLtoys_5Pin_Servo/blob/master/Top.jpg)

Vehicle board bottom side
![](https://github.com/TheDIYGuy999/MRSC_Adapter_WLtoys_5Pin_Servo/blob/master/Bottom.jpg)

Vehicle wiring
![](https://github.com/TheDIYGuy999/MRSC_Adapter_WLtoys_5Pin_Servo/blob/master/Vehicle_wiring.jpg)

First test
![](https://github.com/TheDIYGuy999/MRSC_Adapter_WLtoys_5Pin_Servo/blob/master/wiring.jpg)

(c) 2018 TheDIYGuy999
