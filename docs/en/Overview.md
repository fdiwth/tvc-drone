# Index

1. [Motive](#motive)
2. [Conventions](#conventions)
3. [System](#system)
4. [Control](#control)
5. [Tuning](#tuning)
6. [Features](#features)

# Motive

The drone was made to resemble the dynamics of a rocket in that it produces thrust below the center of mass. However, this design is inherently unstable, and even a small initial tilt will result to the rocket tipping over. An actual rocket overcomes this with mainly two strategies:

1. having the center of pressure lower than the center of gravity at high speeds
2. using a system to actively control the orientation of the rocket

Since the first method can only apply when the rocket moving through the air in the supposed forward direction, it is critical for rockets during the launch or a landing to have a mechanism for stabilizing it.
Once of the most common control mechanisms used is the TVC(Thrust Vector Control). Explained simply, it is a method of stabilizing the rocket by vectoring(changing the direction of thrust). This project focuses on achieving controlled flight using the TVC technology in making a VTVL(Vertical Takeoff Vertical Landing) drone, as a test for the robustness of the TVC control system.
Most of the resources used in this project including 3D CAD models, flight controller PCB, communication protocols, software, was built from scratch to allow full control of the drone's system to the maker.

# Conventions

The drone's coordinate system is shown in the diagram below, along with its associated angles. Roll(x-axis rotation), pitch(y-axis rotation), yaw(z-axis rotation) is used through out the documentation.

# System

The system consists of two havlves:

1. The Drone: Powered by an STM32F722RET microcontroller with dedicated timer triggered FreeRTOS. It is responsible for all time-critical sensor fusion, stabilization, data logging, and actuator commanding.
2. The Ground Control Station(GCS): Powered by an ESP32 microcontroller that interfaces with physical joysticks and a host PC. It manages telemetry decoding, flight mode switching, and relaying operator commands over long-range LoRa RF.

# Control

To achieve stabilization and controlled flight despite the inherently unstable nature of a thrust-vectored vehicle, the flight controller employs a cascading three-level control loop:

1. Position (Outer Loop - P Controller): Looks at the desired 3D coordinates in space and calculates the necessary target velocities to get there.
2. Velocity (Middle Loop - PID Controller): Looks at the target velocities and computes the necessary vehicle tilt (target roll and pitch angles) to accelerate the drone in the correct direction.
3. Attitude (Inner Loop - LQR Controller): Takes the target orientation angles and computes the optimal rapid actuator commands (servo angles and motor thrusts) needed to physically orient the drone without overshooting.

# Tuning

While the LQR controller can calculate mathematically perfect corrections, the physical servos are bound by finite transit times and a 50Hz PWM refresh rate. If the software requests high-frequency oscillations that the servos cannot physically track, it induces a phase lag, leading to catastrophic instability. Thus, the control philosophy heavily relies on constraining the software (clamping max velocity, limiting P-gains, filtering D-terms) to respect the physical bandwidth limits of the hardware.

# Features
Based on the project goals, the following requirements were met in the design:

* Radio communication via LoRa
* Capable of vertical takeoff and landing (VTVL)
* Onboard flash memory and Micro SD for data logging
* Control of 2 counter-rotating motors and 4 gimbal servos
* STM32-based flight controller
* Audiovisual feedback (Buzzer and LEDs)
* External pins for VCC & GND and additional sensors
* Battery voltage and current monitoring
* USB-C interface for debugging and dual-power switching
* Sensor suite: Magnetometer, Accelerometer, Gyroscope, Barometer (Temperature/Pressure)