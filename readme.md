# Smart Car Using IoT README

This repository contains code snippets for various components of an IoT project, including accident alert systems, alcohol detection, motor control, object detection, eye detection, and ultrasonic sensor applications. Below is a brief overview of each component:

## 1. Accident Alert Location Tracking System

### Description
This system utilizes a TTGO T-Call board along with GPS and MPU6050 sensors to detect accidents and send alerts to predefined emergency contacts. It detects accidents based on sudden changes in acceleration and orientation, and sends the location information to contacts via SMS.

### Components
- TTGO T-Call board
- GPS module
- MPU6050 accelerometer and gyroscope
- SIM card for sending SMS alerts

### Libraries Used
- TinyGPS++
- TinyGsmClient
- BlynkSimpleTinyGSM

### Setup Instructions
1. Connect the hardware components as per the pin definitions.
2. Install the required libraries.
3. Update the Blynk authentication token and mobile number in the code.
4. Upload the code to the TTGO T-Call board.
5. Test the system by triggering an accident or using the push button for testing.

## 2. Alcohol Detection System

### Description
This system uses an MQ-3 alcohol sensor to detect alcohol concentration in the air. It measures the voltage output from the sensor and calculates the alcohol concentration in parts per million (ppm). If the concentration exceeds a predefined threshold, it triggers an alert.

### Components
- MQ-3 alcohol sensor
- Arduino board

### Setup Instructions
1. Connect the MQ-3 sensor to the Arduino board.
2. Upload the code to the Arduino board.
3. Test the system by exhaling near the sensor to simulate alcohol presence.

## 3. Motor Control System

### Description
This system allows control of four motors using commands received via serial communication. It supports various commands for motor movement, including forward, backward, left, right, top-left, top-right, bottom-left, bottom-right, and stop.

### Components
- Arduino board
- Four DC motors
- Motor driver (e.g., L298N)

### Setup Instructions
1. Connect the motors and motor driver to the Arduino board.
2. Upload the code to the Arduino board.
3. Send commands via serial communication to control the motors.

## 4. Object Detection and Collision Avoidance

### Description
This system uses ultrasonic sensors to detect objects within a certain range. If an object is detected within the specified distance threshold, it triggers an alarm to alert the user about a potential collision.

### Components
- Ultrasonic sensor
- Buzzer

### Setup Instructions
1. Connect the ultrasonic sensor and buzzer to the Arduino board.
2. Upload the code to the Arduino board.
3. Test the system by placing objects within the detection range of the sensor.

## 5. Eye Detection and Drowsiness Alert

### Description
This system uses a convolutional neural network (CNN) to detect open or closed eyes in real-time using a webcam. If the system detects closed eyes for an extended period, it triggers an alarm to alert the user about drowsiness.

### Components
- Webcam
- Arduino board (for alarm)

### Setup Instructions
1. Connect the webcam to the computer.
2. Install the required Python libraries and dependencies.
3. Run the Python code to start eye detection.
4. Monitor the output for eye status and drowsiness alerts.

## 6. Buzzer Control

### Description
This system controls a buzzer based on commands received via serial communication. It activates the buzzer when it receives a signal indicating closed eyes (indicating drowsiness) and deactivates it when open eyes are detected.

### Components
- Arduino board
- Buzzer

### Setup Instructions
1. Connect the buzzer to the Arduino board.
2. Upload the code to the Arduino board.
3. Send commands via serial communication to activate or deactivate the buzzer.

## 7. Ultrasonic Sensor with Buzzer Alert

### Description
This system combines an ultrasonic sensor and a buzzer to detect objects within a certain range. If an object is detected within the specified distance threshold, it triggers the buzzer to alert the user about a potential collision.

### Components
- Ultrasonic sensor
- Buzzer

### Setup Instructions
1. Connect the ultrasonic sensor and buzzer to the Arduino board.
2. Upload the code to the Arduino board.
3. Test the system by placing objects within the detection range of the sensor.
