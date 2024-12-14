# Romi-Line-Following-Robot
This is a design of a line following robot designed with a STM32 based microcontroller programmed in MicroPython

![image](https://github.com/user-attachments/assets/d2eb8f3c-d84d-4ef7-aa0f-1b8b1be1527a)

## Design
### Mechanical Design 

To allow for the optimal placement of the line and bump sensors, a romi-friendly attachment was designed and fabricated using 3D printing. 

For the line sensor, we tested distances and angles with the best resolution and the least amount of noise. The line sensor is at the same distance radially from the robot as the wheels, which allows for a simple proportional yaw controller from the feedback provided by the line sensor. 

The bump sensors are also kept in front of the line sensor and spaced apart, allowing the Romi to “bump” into the wall and straighten itself out before attempting to circumvent it. This also meant that the mount had to be able to take the impact force from collision, requiring structural rigidity.  The detailed drawing is included in Appendix A.

![image](https://github.com/user-attachments/assets/7e6b0ac0-63c9-472e-9666-9121b3de2a15)

![image](https://github.com/user-attachments/assets/7770e99f-57cf-4829-b8a6-7f3bf5a5d9b2)

### Electrical Design

The main electrical components within the romi line-following robot are a power distribution board receiving six AA batteries in series inputting a voltage between 5.4V and 9V which powers two DC motors and the nucleo-32l476rg microcontroller. 
The microcontroller has dedicated GPIO pins receiving inputs to a eight channel RC based line sensor, two SPDT roller bump sensor switches wired in an active-high configuration, and read signals from two motor encoders. 

This controller also sends outputs as PWM control signals for two DC motors and an I2C line with the BNO055 9-axis absolute orientation sensor. 

A visual of the wiring schematic is listed in figure 3 below. 

![image](https://github.com/user-attachments/assets/55e9c9fc-e0d8-4340-a8ef-9c01f2a5273f)

### Programming Structure

The programming structure for the microcontroller main loop involves task manager based logic sharing variables between tasks and prioritizing tasks with specified period lengths for proper memory allocation. 

Within these tasks are three separate finite state machines as displayed in figures 5-7 to determine, for example, sensor inputs or sending specific control outputs to the motors depending on the state. The color coordination in figure 8 is to help to give a visual of the movement states that are also color coordinated in figure 7 specifying how the robot maneuvers through the obstacle course. 

Included within main.py, are all of the class files for the bump sensors, line sensor, motor drivers, encoder drivers, and the BNO055 sensor where functions are called within the task manager. 

![image](https://github.com/user-attachments/assets/c8509f6c-4956-4eb4-91d6-7afce277ff25)

![image](https://github.com/user-attachments/assets/d1d154f4-f8c1-4985-979e-f74d7ea55dd8)

![image](https://github.com/user-attachments/assets/9d590a1f-e2fa-44f6-a061-40989842a2be)

![image](https://github.com/user-attachments/assets/f5b3ee0d-4e79-4a5e-aefd-d1f501b14cdb)

![image](https://github.com/user-attachments/assets/3ce65cc0-a636-4054-9ce1-13343fd35f54)

## Appendix
### Appendix A: Detailed Drawing of Sensor Array

![image](https://github.com/user-attachments/assets/b9f6d3fa-27e4-4acc-a1dc-1ba93b7508db)

### Appendix B: Video Demonstration Link








