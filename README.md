# Romi-Line-Following-Robot
This is a design of a line following robot designed with a STM32 based microcontroller programmed in MicroPython

## Design
### Mechanical Design 

To allow for the optimal placement of the line and bump sensors, a romi-friendly attachment was designed and fabricated using 3d printing. 

For the line sensor, we tested distances and angles with the best resolution and the least amount of noise. The line sensor is at the same distance radially from the robot as the wheels, which allows for a simple proportional yaw controller from the feedback provided by the line sensor. 

The bump sensors are also kept in front of the line sensor and spaced apart, allowing the Romi to “bump” into the wall and straighten itself out before attempting to circumvent it. This also meant that the mount had to be able to take the impact force from collision, requiring structural rigidity.  The detailed drawing is included in Appendix A.

![image](https://github.com/user-attachments/assets/7e6b0ac0-63c9-472e-9666-9121b3de2a15)

Add figure 2 here

### Electrical Design

The main electrical components within the romi line-following robot are a power distribution board receiving six AA batteries in series inputting a voltage between 5.4V and 9V which powers two DC motors and the nucleo-32l476rg microcontroller. 
The microcontroller has dedicated GPIO pins receiving inputs to a eight channel RC based line sensor, two SPDT roller bump sensor switches wired in an active-high configuration, and read signals from two motor encoders. 

This controller also sends outputs as PWM control signals for two DC motors and an I2C line with the BNO055 9-axis absolute orientation sensor. 

A visual of the wiring schematic is listed in figure 3 below. 

Add figure 3 here

## Appendix
## Appendix A: Detailed Drawing of Sensor Array

Place image here

