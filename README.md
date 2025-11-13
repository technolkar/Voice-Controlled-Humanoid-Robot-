# Voice-Controlled-Humanoid-Robot-
Voice Controlled Humanoid Robot is mainly built to Identify the voice recognizing system in the machines and control the robot using Google assistant through the mobile.


# Project Overview

This project showcases the design and fabrication of a functional humanoid robot developed as part of our final-year Diploma in Mechatronics Engineering at M.L. Bharatesh Polytechnic, Belagavi (India). The primary goal was to create a human-like robot capable of movement, object manipulation, and basic communication, controlled through IoT technology and simple automation features. The project was designed, built, and tested by a team of nine students under the guidance of Mr. Amey Sir and Mr. Suraj Sir, over a development period of eight months.

# Concept & Objective

The aim of this project was to build a semi-autonomous humanoid robot that can:
Assist humans through IoT-based remote operation
Perform pick-and-place tasks with a robotic hand
Interact verbally through a pre-programmed voice system
Move and navigate using line-following and obstacle-avoidance systems
This robot demonstrates how mechanical design, electronics, and IoT control can be integrated to achieve intelligent human-like behavior at low cost.

## ⚙️ Technical Specifications

| **Component** | **Description** |
|----------------|----------------|
| **Microcontrollers** | Arduino UNO and NodeMCU (ESP8266) |
| **Motor Drivers** | L298 Dual H-Bridge |
| **Servo Control** | 16-Channel PWM Servo Controller |
| **Communication** | IoT (Blynk App + IFTTT Integration) |
| **Sensors** | Ultrasonic Sensor (HC-SR04), IR Line-Following Sensors |
| **Actuators** | Servo Motors (for arms, fingers, and head), DC Motors (for base movement) |
| **Power Supply** | 12V DC battery with voltage regulation |
| **Body Design** | Modified mannequin structure with four-wheel-drive mobile base |
| **Programming Tools** | Arduino IDE, Blynk IoT App, IFTTT Webhooks |
| **Connectivity** | Wi-Fi-based IoT control via smartphone app |
| **Voice System** | Pre-recorded responses using voice playback module |
| **Camera Module** | Mounted USB/ESP32-CAM for remote surveillance |

# Features


🧍‍♂️ **Humanoid Motion**

Full 360° head and hip rotation.  
Arm and finger movements powered by servo motors.  
Capable of pick-and-place operations for lightweight objects.

🦾 **IoT Control**

Controlled remotely using a smartphone IoT application (Blynk).  
Wireless communication via NodeMCU (ESP8266).  
User can move the robot base, rotate joints, and operate arms in real time.

🧭 **Autonomous Navigation**

Line-following system using IR sensors.  
Obstacle avoidance with ultrasonic distance sensing.  
Ability to switch modes between IoT control and autonomous driving.

🗣️ **Voice Interaction**

Integrated voice playback system that responds to predefined questions.  
Acts like a basic talk-back assistant for interactive demonstrations.

🎥 **Surveillance Camera**

Mounted camera module for live video streaming.  
Enables remote monitoring of surroundings through the IoT app.

# 🔧 Development Process

**Mechanical Design**

Used a male mannequin as the robot’s physical structure.

Added a four-wheel DC motor base for movement.

Designed arms, fingers, and joints to move through servo actuation.

Fabricated brackets and mounts using metal and acrylic sheets.

**Electronics Integration**

Assembled and soldered all control circuits manually.

Configured Arduino UNO for motion and servo control.

Interfaced NodeMCU (ESP8266) for wireless IoT connectivity.

Integrated IR and ultrasonic sensors for navigation and obstacle detection.

Added a voice module for interactive talk-back responses.

**Software Implementation**

Programmed using Arduino IDE with modular functions for motion, voice, and IoT tasks.

Developed a custom Blynk IoT dashboard for remote control via smartphone.

Calibrated all servos and motors for smooth, synchronized movement.

**Testing & Optimization**

Conducted repeated field tests for motion accuracy and communication stability.

Tuned servo angles for natural human-like movement.

Debugged issues in power distribution, Wi-Fi range, and signal interference.

Verified all operating modes — IoT control and autonomous navigation.

**Challenges Faced**

Limited funding required creative reuse of components and low-cost solutions.

Frequent motor burns, wiring faults, and circuit instability during early tests.

Difficulty maintaining Wi-Fi connectivity for consistent IoT response.

Synchronizing multiple systems (IoT, voice, sensors, and motion) without delays.

Despite these hurdles, continuous teamwork and persistence led to a fully functional humanoid prototype.

# Circuit Diagram 

![base circuit diagram design](Slide10.JPG)

Base Circuit Design

This circuit controls the robot’s base movement, enabling line following, obstacle avoidance, and IoT-controlled driving.

Main Components

Arduino UNO (Left Section) – Controls line following & obstacle avoidance

NodeMCU (Right Section) – Enables IoT-based control through a mobile app

L298 Motor Drivers (2 Units) – Drive the DC motors on the 4-wheel base

IR Sensors – Detect black and white lines for line following

Ultrasonic Sensor – Detects obstacles and measures distance

12V Batteries (2 Units) – Power supply for Arduino, NodeMCU, and motor drivers

🔹 Connection Breakdown
🧭 Line Following and Obstacle Avoidance (Left Section – Arduino UNO)

IR Sensors → Arduino UNO:

The OUT pins of the two IR sensors connect to the digital pins (e.g., D2, D3) of the Arduino.

Used to sense the track (black/white line).

Ultrasonic Sensor → Arduino UNO:

Trig pin → D9, Echo pin → D10 of Arduino.

Measures distance and prevents collision with objects.

Arduino UNO → L298 Motor Driver (Left):

Digital pins D5, D6, D7, D8 control IN1–IN4 of the L298 driver.

Enables direction and speed control of two DC motors (left side).

L298 Motor Driver → Motors:

Two DC motors are connected at Output1 & Output2 terminals.

Power Supply:

12V battery connected to L298’s Vcc and Arduino’s Vin (via regulated input).

📶 IoT-Based Base Control (Right Section – NodeMCU)

NodeMCU → L298 Motor Driver (Right):

Digital pins D1, D2, D3, D4 connect to IN1–IN4 of the second L298 driver.

Controls the second pair of DC motors (right side).

Power Supply:

Separate 12V battery powers the L298 driver and NodeMCU (through onboard 5V regulator).

⚡ Working Principle

The Arduino handles autonomous functions (line following or obstacle avoidance).

The NodeMCU takes over when IoT mode is enabled, controlling the motors wirelessly via the mobile app.

Only one mode (IoT or Autonomous) is active at a time, selected by a switch or program condition.


# 🏆 Achievements

Presented at multiple National-Level Technical Events.

Secured Second Prize at Aavishkar National-Level Technical Fest.

Appreciated by the Department of Mechatronics for innovation and integration.

# 💡 Learning Outcomes

Hands-on experience in IoT integration, robotics motion control, and sensor interfacing.

Strong understanding of embedded system design and human–robot interaction.

Enhanced teamwork, leadership, and problem-solving abilities through real-world prototyping.

# 🧰 Future Improvements

Integrate AI-based speech recognition and computer vision for intelligent interaction.

Upgrade control to Raspberry Pi with ROS for higher autonomy and mapping.

Implement self-charging and path-planning (SLAM) for full autonomous mobility.

Add gesture and voice control with cloud connectivity for remote teleoperation.

# 👥 Team Members

Ajay Mannolkar — Team Leader  
Nilesh Chougule  
Mahadev Jadhav  
Mallikarjun Madapaki  
Kaustubh Melge  
Amrut Marihal  
Bikashmurthy Bhandi  
Praveen Naik  
Vaibhav Shreyakar  



Guides: Mr. Amey Sir and Mr. Suraj Sir


