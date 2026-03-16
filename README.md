[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/-Acvnhrq)
# Final Project

**Team Number:** 10

**Team Name:** ???

| Team Member Name | Email Address       |
|------------------|---------------------|
| Cindy Liu        | [cinl@seas.upenn.edu]           |
| Mike Song        | [chengyus@seas.upenn.edu]           |
| Justin Yu        | [justinyu@seas.upenn.edu]           |

**GitHub Repository URL:** https://github.com/upenn-embedded/final-project-s26-t10 

**GitHub Pages Website URL:** [for final submission]*

## Final Project Proposal

### 1. Abstract
This project is the design and implementation of a 2-axis camera gimbal system that uses servos to provide active stabalization for a camera. It uses an Inertial Measurement Unit (IMU) to monitor angular rotation as an input. Feedback is processed by an ATMega328PB. The microcontroller will output PWM signals, corrected by a PID control algorithm. Finally, the entire system will be powered by USB-C.

### 2. Motivation
Conventional camera gimbals are expensive, usually upwards of $200. Furthermore, they are usually designed specifically for industry grade cameras, leaving much to be desired for amateur videographers. This project gives an affordable option for those looking to try recording videos wth stabalization. By utilizing the widespread USB-C connector for power, this device can be used in many environments with a variety of power sources.

### 3. System Block Diagram
![alt text](<Images/System Block Diagram.drawio.png>)

### 4. Design Sketches
![design sketch](./Images/design%20illustration.png)

### 5. Software Requirements Specification (SRS)

**5.1 Definitions, Abbreviations**

(IMU) - Inertial Measurement Device, gives linear acceleration and angular rate data \
(Angular Rate) - degrees per second, as given by IMU \
(PID controller) Proportional-Integral-Derivative controller. Uses feedback to hit a target while reducing overshoot, in this case to stabalize the axis after rotation. Uses integral and derivative of error to adjust output.\
(PWM) - pulse width modulation, produces a square wave output from the microcontroller to serve as an input to the servo. 

**5.2 Functionality**

| ID     | Description                                                                                                                                                                                                              |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| SRS-01 | The IMU 2-axis angular rate will be measured with 16-bit depth every 5 milliseconds +/-1 milliseconds. |
| SRS-02 | The PID controller will recieve new inputs and produce PWM outputs for each servo every 5 milliseconds +/-1 milliseconds, based on estimated error with an accumulator variable that sums IMU's angular rate every polling cycle. |
| SRS-03 | The PWM Duty cycle that is output to the servos will be updated every 5 milliseconds +/-1 milliseconds.|
| SRS-04 | The gimbal will read the input of an "enable" button so it will stop stabalizing if the button is pressed. |
| SRS-05 | The device can be reset to zeroed positions with the press of the "zero" button.  |
| SRS-06 | The device can be manually zeroed and the position can then be remembered with a long press of the "zero" button. |
| SRS-07 | The user shall be able to determine the state of the device operation by viewing the status LED. |


### 6. Hardware Requirements Specification (HRS)

**6.1 Definitions, Abbreviations**

MCU - Microcontroller Unit, compact integrated circuit that processes inputs and manages dedicated outputs and tasks.
Camera Plate - Platform for GoPro/similar sized camera to rest on, controlled on 2 axes by the servos.
Zeroing - Setting the default position for the camera plate, manually done by aligning the camera plate with the handle.

**6.2 Functionality**

| ID     | Description                                                                                                                        |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | USB-C PD module should deliver 12V output from an USB-C input. |
| HRS-02 | Buck Converter Module should deliver stable 5V output. |
| HRS-03 | ATmega328PB will function as the microcontroller for the device and also deliver 3.3V. |
| HRS-04 | 2 IMUs interfaced via I2C will deliver acceleration and position data to the MCU. |
| HRS-05 | 2 servos will control the pitch and roll of the camera plate.  |
| HRS-06 | Servos combined with IMU inputs stabilize movement of camera plate in two degrees of freedom. |
| HRS-07 | Two buttons that allows users to enable the gimbal function and zero the servos. |

### 7. Bill of Materials (BOM)

https://docs.google.com/spreadsheets/d/1yj54xOVig_wChPn7wCOqRE96ZmyI3QeAy7MwsV0c4z8/edit?gid=2071228825#gid=2071228825

### 8. Final Demo Goals

Demonstrate a 2-axis (pitch and roll) active stabilization. Able to keep camera platform stable despite hand movements. The system shall read angular velocity from an IMU, compute a stable angle estimate, and drive two sevos. Independent PID controllers on each axis shall correct for angular error at 200Hz.

### 9. Sprint Planning

| Milestone  | Functionality Achieved | Distribution of Work |
| ---------- | ---------------------- | -------------------- |
| Sprint #1  | Using an I2C Library, stabilize pitch, create handle and first servo mount | Mike will CAD the handle and servo mount. Justin and Cindy will implement and tune PID for one axis. |
| Sprint #2  | Move away from using I2C library, include second servo and begin tuning both axes in tandem. Begin FOC bring-up on pitch motor. | Mike will design and integrate the second servo mount and validate mechanical balance on both axes. Cindy will work on integration between the 2 axes and the PID loop. Justin will write the bare-metal SPI communications protocol to replace the I2C library. |
| MVP Demo   | Both axes stabilizing with servo actuation. Custom SPI driver operational. Complementary filter producing stable angle estimates on both axes. PID visibly rejecting hand-induced disturbances. Mechanical assembly fully integrated. | Mike ensures full mechanical assembly is complete and camera platform is balanced. Cindy integrates both PID loops and validates stable closed-loop behavior on pitch and roll. Justin validates I2C driver and verifies clean IMU data. |
| Final Demo | Custom I2C driver, complementary filter, PID controllers, Clarke/Park transforms, and SVM all written from scratch in register-level C. System visibly stabilizes camera footage under moderate hand disturbance at 200Hz. | Mike finalizes mechanical assembly, performs cable management, and captures demo footage. Justin and Cindy tunes PID gains on both axes, implements deadband, and optimizes control loop timing.  |

**This is the end of the Project Proposal section. The remaining sections will be filled out based on the milestone schedule.**

## Sprint Review #1

### Last week's progress

### Current state of project

### Next week's plan

## Sprint Review #2

### Last week's progress

### Current state of project

### Next week's plan

## MVP Demo

## Final Report

Don't forget to make the GitHub pages public website!
If you’ve never made a GitHub pages website before, you can follow this webpage (though, substitute your final project repository for the GitHub username one in the quickstart guide):  [https://docs.github.com/en/pages/quickstart](https://docs.github.com/en/pages/quickstart)

### 1. Video

### 2. Images

### 3. Results

#### 3.1 Software Requirements Specification (SRS) Results

| ID     | Description                                                                                               | Validation Outcome                                                                          |
| ------ | --------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| SRS-01 | The IMU 3-axis acceleration will be measured with 16-bit depth every 100 milliseconds +/-10 milliseconds. | Confirmed, logged output from the MCU is saved to "validation" folder in GitHub repository. |

#### 3.2 Hardware Requirements Specification (HRS) Results

| ID     | Description                                                                                                                        | Validation Outcome                                                                                                      |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | A distance sensor shall be used for obstacle detection. The sensor shall detect obstacles at a maximum distance of at least 10 cm. | Confirmed, sensed obstacles up to 15cm. Video in "validation" folder, shows tape measure and logged output to terminal. |
|        |                                                                                                                                    |                                                                                                                         |

### 4. Conclusion


## References

