# FTC-Autonomous-Opmode

This project contains an autonomous opmode for a robot competing in the FIRST Tech Challenge (FTC). The opmode is designed to perform complex movements while maintaining accurate position tracking using odometry and sensor fusion techniques.

## Overview
The opmode is built using a combination of several advanced mathematical concepts, including multivariable calculus, arc length parametrization, Kalman filtering, and sensor fusion. By incorporating these techniques, the robot is able to accurately track its position and orientation on the field, and perform complex movements such as arcs and circles.

The opmode is split into several classes, each of which handles a specific aspect of the robot's behavior. Robot.java defines the physical characteristics and behaviors of the robot, while Odometry.java uses calculus to accurately track its position and orientation. SensorFusion.java combines data from the robot's IMU and odometry sensors to provide a more accurate estimation of its position and orientation, while CurvedMovement.java adds the ability to perform curved movements. AutonomousMasterCode.java ties together all of these components to create a comprehensive autonomous opmode.

## Usage
To use these programs, you first need to make sure that the robot's hardware components are connected and initialized correctly in the Robot.java file. This includes setting up the motor controllers, sensors, and other devices according to your robot's configuration.

Once the hardware is set up, you can use the following programs in sequence to enable advanced autonomous capabilities:

**Odometry.java**: This program initializes the robot's starting position and tracks its movement using wheel encoders. Running this program first will provide a starting point for subsequent programs that rely on accurate position tracking.

**SensorFusion.java**: This program fuses data from the wheel encoders and the IMU to obtain more accurate and reliable position and heading estimates. Running this program after Odometry.java will improve the accuracy of subsequent autonomous routines.

**CurvedMovement.java**: This program plans and executes curved movements by generating a smooth path and controlling the robot's speed and turning using a PID controller. You can use this program to perform complex maneuvers such as turns and arcs.

**AutonomousMasterCode.java**: This program allows you to create and execute custom autonomous routines, incorporating the previous programs as necessary. Use the provided functions and variables to control the robot's movement and integrate additional sensors and functions as needed.

When using these programs, it's important to constantly monitor the robot's position and heading to ensure it is following the desired path and performing the correct actions. Use the telemetry output and debugging tools available in the FTC SDK to aid in this process.

## Dependencies
This opmode requires the following components:

FTC SDK
Java SE Development Kit (JDK) 8+
Robot controller phone
Expansion hub
Motors and servos
Encoder sensors
IMU sensor

## Credits
This code was developed by chat gpt and Nathan Tam as part of FTC Team #11208. It incorporates advanced mathematical concepts and techniques to improve the accuracy and performance of the robot.

## License
This project is licensed under the [insert license]. You are free to use, modify, and distribute this code as long as you adhere to the terms of the license.
