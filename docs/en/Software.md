# Index

1. [Controls](#controls)
	- [LQR](#lqr)
	- [PID](#pid)
	- [Anti-Windup Mechanisms](#anti-windup-mechanisms)
	- [LQR Servo Mapping](#lqr-servo-mapping)
2. [Sensor fusion](#sensor-fusion)
	  - [Orientation Fusion](#orientation-fusion)
	  - [Altitude Fusion](#altitude-fusion)
	  - [Velocity Fusion](#velocity-fusion)
	  - [Low-pass Filter](#low-pass-filter)
	  - [Sensor Calibration](#sensor-calibration)
3. [Communication](#communication)
	  - [LoRa Configuration](#lora-configuration)
	  - [Asymmetric Communication Protocol](#asymmetric-communication-protocol)
	  - [Packet Structure](#packet-structure)
4. [FreeRTOS](#freertos)
	  - [Flight State](#flight-state)

# Controls

There are mainly four different ways to make a TVC system: movable fins, gimbaled thrust, vernier rocket, and thrust vanes.

<div align="center">
	<img src="../assets/examples-of-controls.gif" width="400" />
</div>

However, the most modern rockets like SpaceX's falcon 9 and Starship, Rocket Lab's Electron, NASA's SLS,  KARI's Nuri(KSLV-II), JAXA's H3, CASC's Long March, ESA's Ariane and so on use gimbaled thrust to achieve controllability during flight. 

Overall, the controllable values of the drone are the pwm signals for servo-x, servo-y, motor1, and motor2. With these, the drone is able to control its orientation and position around all x, y and z axis. 
### **LQR**

A control scheme used in controlling the orientation and altitude of the drone is the **LQR (Linear Quadratic Regulator)**. This uses state vectors reflecting the current state of the system and matrices that are pre-computed using the properties of the system to drive the actuators in the direction that minimizes error. Because the gain matrix is pre-computed to be optimal, it bypasses the tedious process of manual tuning and only requires bench tests to balance the gains for different types of error and actuation penalty.

The controller operates on a 12-state augmented vector consisting of orientation/altitude errors, their derivatives, and their integrals:

$$
\mathbf{x} = \begin{bmatrix} r_x & r_y & r_z & p_z & \dot{r}_x & \dot{r}_y & \dot{r}_z & \dot{p}_z & \int r_x & \int r_y & \int r_z & \int p_z \end{bmatrix}^T \in \mathbb{R}^{12}
$$

where $r_x, r_y, r_z$ are roll/pitch/yaw orientation errors, $p_z$ is the altitude (position-z) error, the $\dot{(\cdot)}$ terms are their rates, and the $\int(\cdot)$ terms are their accumulated integrals, which are added for steady-state error rejection.

The system is modeled as a linear time-invariant plant:

$$
\dot{\mathbf{x}} = A_c \mathbf{x} + B_c \mathbf{u}
$$

with the state matrix $A_c$ encoding the kinematic relationships:

$$
A_c =
\left[\begin{array}{cccccccccccc}
0&0&0&0&1&0&0&0&0&0&0&0\\
0&0&0&0&0&1&0&0&0&0&0&0\\
0&0&0&0&0&0&1&0&0&0&0&0\\
0&0&0&0&0&0&0&1&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
1&0&0&0&0&0&0&0&0&0&0&0\\
0&1&0&0&0&0&0&0&0&0&0&0\\
0&0&1&0&0&0&0&0&0&0&0&0\\
0&0&0&1&0&0&0&0&0&0&0&0
\end{array}\right]
$$

and the input matrix $B_c$ mapping the actuation vector $\mathbf{u} = \begin{bmatrix} u_0 & u_1 & u_2 & u_3 \end{bmatrix}^T$ (servo-x, servo-y, differential thrust, main thrust) into the rate states, scaled by the physical control-effectiveness constants $B_{rx}$, $B_{ry}$, $B_{rz}$, $B_{pz}$. Values for $B_{rx}$ and $B_{ry}$ were determined using bifilar pendulum to calculate moment of inertia about the CG at x and y axis, $B_{rz}$ was determined empirically by testing the angular acceleration about the z axis at various rpm differences of motors. Lastly, $B_{pz}$ was simply calculated using the drone's final mass.

$$
B_c =
\left[\begin{array}{cccc}
0&0&0&0\\
0&0&0&0\\
0&0&0&0\\
0&0&0&0\\
-B_{rx}&0&0&0\\
0&B_{ry}&0&0\\
0&0&B_{rz}&0\\
0&0&0&B_{pz}\\
0&0&0&0\\
0&0&0&0\\
0&0&0&0\\
0&0&0&0
\end{array}\right]
$$

Since the flight controller runs at a fixed control-loop rate, the continuous plant is converted to discrete time via zero-order hold with sample time $\Delta t$:

$$
\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d \mathbf{u}_k, \qquad (A_d, B_d) = \mathrm{ZOH}(A_c, B_c, \Delta t)
$$

The LQR gain is found by minimizing the infinite-horizon quadratic cost:

$$
J = \sum_{k=0}^{\infty} \left( \mathbf{x}_k^T Q \, \mathbf{x}_k + \mathbf{u}_k^T R \, \mathbf{u}_k \right)
$$

where $Q \in \mathbb{R}^{12\times12}$ penalizes state error (orientation, rate, and integral terms) and $R \in \mathbb{R}^{4\times4}$ penalizes actuator effort:

$$
Q = \text{diag}\big(q_{r_x},\, q_{r_y},\, q_{r_z},\, q_{p_z},\, q_{\dot r_x},\, q_{\dot r_y},\, q_{\dot r_z},\, q_{\dot p_z},\, q_{\int r_x},\, q_{\int r_y},\, q_{\int r_z},\, q_{\int p_z}\big)
$$

$$
R = \text{diag}\big(r_{u_0},\, r_{u_1},\, r_{u_2},\, r_{u_3}\big)
$$

The optimal cost-to-go matrix $P$ is obtained by solving the DARE:

$$
P = A_d^T P A_d - A_d^T P B_d \left( R + B_d^T P B_d \right)^{-1} B_d^T P A_d + Q
$$

This can be solved by using either python(`lqr_compute.py`) or matlab (`lqr_compute.m`)

The optimal feedback gain matrix is then computed directly from $P$:

$$
K = \left( R + B_d^T P B_d \right)^{-1} B_d^T P A_d
$$

and the control input applied at each time step is the linear feedback law:

$$
\mathbf{u}_k = -K \, \mathbf{x}_k
$$

Because $K$ is solved once for the full 12-state augmented system, the resulting gains for the servo, differential-thrust, and thrust-vectoring channels are already balanced against one another. This eliminates the need for manual PID-style tuning and leaving only bench testing to fine-tune the relative weighting between error suppression and actuation cost. 
### **PID**

Most drone use the PID(Proportional Integral Derivative) controlling scheme to maintain stability during flight. Where the control output regarding the error is actuated in a tuned factor to each of proportional, integral and derivative gains. The proportional term steers the system towards reducing the error, the integral term accounts for inaccuracies in the system, and finally the derivative term acts as a damping to the otherwise oscillating system. Here is a great video on it: https://youtu.be/4Y7zG48uHRo?si=Jpp3Fp_0xEZwwcJJ.

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{d e(t)}{dt}$$

Because the integral and derivative has to be calculated computationally in a discrete time the function becomes:

$$u(t_n) = K_p e(t_n) + K_i \sum_{k=0}^{n} e(t_k) \Delta t + K_d \frac{e(t_n) - e(t_{n-1})}{\Delta t}$$

The XY (horizontal) position was controlled using a **cascaded structure**. In this cascaded structure, the target XY orientation is decided by a PID controller regulating XY velocity. The target XY velocity, in turn, is decided by a P controller regulating XY position errors.

1. Position Error  Target Velocity (P Controller): The outermost loop calculates the difference between the desired target coordinate and the current 3D position. A simple proportional (P) controller converts this distance error into a target velocity. This prevents the drone from attempting to instantly teleport to the target, instead moving at a controlled speed.
   - Tuning ($K_p$ = 0.5): The firmware utilizes a highly conservative gain of $0.5$ for both the X and Y axes.
   - Clamping: The output of this stage is strictly clamped to $\pm0.5$ m/s to prevent erratic, high-speed lunges during large position changes.

1. Velocity Error $\rightarrow$ Target Attitude (PID Controller): The middle loop compares the current estimated velocity (from the Optical Flow and IMU fusion) against the target velocity provided by the Position loop. It uses a full Proportional-Integral-Derivative (PID) controller to output a target roll/pitch angle. To accelerate forward, the drone must pitch forward.
   - Tuning ($K_p$ = 0.140, $K_i$ = 0.040, $K_d$ = 0.008): Notice that the gains here are much smaller than the outer loop.
   - Clamping: The target attitude output is hard-clamped to $\pm0.2$ radians ($\approx 11.4^\circ$). This hard limit prevents the drone from ever commanding an extreme tilt that could compromise lift or cause an unrecoverable flip.

<div align="center">
	<img src="../assets/control-architecture.png" width="800" />
</div>

The position control is essential if one wants to prevent drift of the drone. Because the drone produces thrust below its center of gravity, it is dynamically similar to an inverted pendulum. It is inherently unstable and wants to tip over. Furthermore, tilting the gimbal in one direction not only causes a rotational torque but also induces translational motion. If there are slight misalignments in the CG or servos, the system will slowly drift over time. 

As mentioned, the XY controller is split into an inner PID controller for velocity and an outer P controller for position. This structure allows the inner controller to quickly absorb any systematic mechanical offsets with its Integral (I) gain, while allowing the Proportional (P) gain of the outer position loop to be adjusted independently.
### **Anti-Windup Mechanisms**

Because the control architecture is cascaded (Position $\rightarrow$ Velocity $\rightarrow$ Attitude), a crucial issue arises: integral windup. If the drone is pushed hard and the servos reach their physical maximum angle (saturation), the position and velocity errors will continue to grow because the drone cannot physically tilt any further to correct it. Without safeguards, the Integral term in the velocity loop would accumulate a massive, unrealistic error value. Once the drone is released, this huge integral value would cause the drone to violently snap back in the opposite direction.

To solve this, the firmware implements an intelligent **Anti-Windup mechanism** in `freertos.c`. The system continuously monitors the downstream outputs (servo angles and motor thrusts). If the LQR attitude controller reaches its maximum allowed limits, a signal is passed back up the chain to the velocity PID controllers to *freeze* their integral accumulators. The integrals are only allowed to grow when the system has the physical authority to act upon them.
### **LQR Servo Mapping**

The LQR output vector ($u$) produces normalized control efforts. These values cannot be written directly to the hardware. Instead, they are translated into specific PWM signals for the two gimbal servos and two brushless motors through a mixing matrix.
- Base Offset (`LQR_SSO = 50`): The neutral, upright position for the servos is defined as an percentage of 50% of actuation range. The LQR pitch and roll efforts are scaled and dynamically added/subtracted from this base offset.
- Differential Thrust: Because the drone lacks a physical yaw servo, the LQR yaw effort ($u_2$) is mapped to a thrust differential. A positive yaw command adds thrust to Motor 1 while equally subtracting it from Motor 2, generating a rotational torque without affecting the total altitude lift.
# Sensor fusion

The most important part of a controls system is the sensors. If a system has no idea on what state it is at, it does not have the ability to steer itself towards the targeted state. First, the drone needs to know how it is oriented and second, it needs to know where it's positioned at. There are many sensors aboard the PCB and there are other sensors that are connected to the PCB via connectors that allow the drone to function properly. Some sensors only work in limited environments, some might be prone to drifting over time, and other might have too much noise in their readings. To overcome these problems, sensor readings have to be fused in different ways to obtain an accurate state of the system.


#### **Orientation Fusion**

The orientation of the drone is derived the sensor data is achieved with a 6-axis IMU(Inertial measurement unit) and a magnetometer. The Madgwick filter from xioTechnologies fusion library was used to fuse the sensor data together. The madgwick filter uses the accelerometer's xyx values to estimates the direction of gravity, and combines it with a smoother gyroscope readings to obtain orientation values. However, since direction of gravity is the same regardless of z-axis orientation, using the IMU alone cannot give a accurate z-axis reading. A magnetometer which senses the direction of Earth's magnetic field has to be used to as the absolute reference to the heading. A more mathematical explantion of the Madgwick filter can be found here: https://ahrs.readthedocs.io/en/latest/filters/madgwick.html

To obtain z-position(altitude), barometer, ToF(Time of Flight) sensor and accelerometer values from IMU were used. Barometer measures the atmospheric pressure at the current position. Since atmospheric pressure decreases with height according to the following formula:

$$
P = P_0 \cdot \left(1 - \frac{L \cdot h}{T_0}\right)^{\frac{g \cdot M}{R \cdot L}}
$$
$$
\frac{P}{P_0} = \left(1 - \frac{L \cdot h}{T_0}\right)^{\frac{g \cdot M}{R \cdot L}}
$$
$$
\left(\frac{P}{P_0}\right)^{\frac{R \cdot L}{g \cdot M}} = 1 - \frac{L \cdot h}{T_0}
$$
$$
\frac{L \cdot h}{T_0} = 1 - \left(\frac{P}{P_0}\right)^{\frac{R \cdot L}{g \cdot M}}
$$
$$
h(P) = \frac{T_0}{L} \cdot \left(1 - \left(\frac{P}{P_0}\right)^{\frac{R \cdot L}{g \cdot M}}\right)
$$

Standard Temperature ($T_0$) = $288.15 \text{ K}$
Lapse Rate ($L$) = $0.0065 \text{ K/m}$
Gravity ($g$) = $9.80665 \text{ m/s}^2$
Molar Mass of Air ($M$) = $0.0289644 \text{ kg/mol}$
Gas Constant ($R$) = $8.31432 \text{ J/(mol}\cdot\text{K)}$

Plugging these number yields: 

$$
h(P) = 44330.0 \cdot \left(1.0 - \left(\frac{P}{100.0 \cdot P_0}\right)^{\frac{1}{5.255}}\right)
$$

by which altitude can be calculated.

The ToF sensor has a limited range of up to 2m above a surface but provides much more accurate altitude readings. It uses the time it takes for light to travel to the ground surface and back to calculate the altitude. Since the ToF sensor is attatched physically to the drone, it tilts with the drone creating larger altitude readings than reality. This can be compensated for by scaling the reading by cosine of x and y orientations. 
### **Altitude Fusion**

Linear acceleration about the absolute Z-axis, altitude derived from the barometer, and the altitude readings from the compensated ToF sensor are fused using a Kalman filter. 

- **Under 2 meters (ToF in range)**: The ToF sensor acts as the absolute authority due to its millimeter-level precision. During this phase, the Kalman filter trusts the ToF heavily and simultaneously uses the ToF readings to continuously recalibrate the Barometer's ground offset.
- **Over 2 meters (ToF out of range)**: Once the drone exceeds the 2m threshold, the ToF reading becomes unreliable. The Kalman filter hands off primary altitude estimation to the barometer. Because the barometer was recalibrated just before leaving the 2m zone, the transition is seamless without sudden jumps in estimated altitude.
### **Velocity Fusion**

X and y velocities are sensed with an optical flow sensor (PMW3901) and integrated into position values. The optical flow sensor tracks patterns on ground surfaces to output angular velocity. 

Since the optical flow sensor is physically mounted to the drone body, it senses a false change in surface patterns simply when the drone tilts. Thus, the angular velocity derived from the gyroscope orientation readings must be subtracted from the optical flow readings. Another effect is parallax: closer objects seem to move faster than objects further away. The raw readings are scaled based on the current estimated altitude to obtain the true velocity. This is then fused with accelerometer values using Kalman filters for each axis.

### **Low-pass Filter**

A discrete low pass filter of form:

$$y_{n} = \alpha \cdot y_{n-1} + (1 - \alpha) \cdot x_{n}$$

was used in appropriate places to filter out noise.

### **Sensor Calibration**

Sensors need to be calibrated before use, to account for any misalignments or offsets. Magnetometer was calibrated for both soft iron and hard iron errors by using the MotionCal application to derive a 3 by 3 calibration matrix. And accelerometers are pre-calibrated to account for the PCB mounting offset and the sensor's inherent misalignments. Gyroscopes are calibrated at the beginning of every starting sequence when the drone gets power. 

# Communication

The drone communicates with the receiver using LoRa RF modules. Specifically, two **Ebyte E220-900T22D** modules are used—one onboard the drone and one at the ground station—to transmit and receive data, telemetry, and flight commands over long distances.

![[communication-diagram.png]]


###  **LoRa Configuration**

To comply with regional frequency regulations (e.g., Korea allows 920.9 ~ 923.3 MHz) and optimize for range, the Ebyte modules are configured using the following parameters:

| Setting | Value | Description |
| -------------------|-----------| -------------|
| Center Frequency   | 922 MHz   | Legal operating band |
| Air Data Rate      | 19.2 kHz  | Balances throughput and transmission distance |
| Baudrate           | 115200    | UART speed between the module and MCU |
| Parity Bit         | 8N1       | 8-bit data, no parity, 1 stop bit |
| Sub-packet Length  | 200 bit   | Buffer size for packet splitting |
| Transmission Power | 10 dBm    | Output strength |
| LBT (Listen Before Talk) | Enabled | Required by regional regulations to prevent collision |

To enter configuration mode, the M0 and M1 pins on the Ebyte module must be pulled high. The configuration commands follow the format: `command + address + length + parameters`.

Important registers to configure are:
- **REG0**: UART baud rate, Air data rate, Serial parity bit.
- **REG1**: Sub-packet length, Transmission power.
- **REG2**: Channel frequency (850.125 + REG2 MHz).
### **Asymmetric Communication Protocol**

<div align="center">
	<img src="../assets/asymmetric-protocol.png" width="400" />
</div>

A custom communication protocol, loosely modeled off of standard packet-based networking, was implemented to make the connection robust and minimize overhead. 

The communication architecture is asymmetric:
1.  Drone to GCS (Downlink): Telemetry data sent from the drone to the GCS requires high throughput and reliability. It follows a custom binary protocol, transmitting tightly packed sensor readings and status flags in raw binary to minimize packet size.
2.  GCS to Drone (Uplink): Commands sent from the GCS to the drone (e.g., arm, disarm, calibrate) are less frequent and simply use standard UTF-8 string formats.
#### **Packet Structure**

<div align="center">
	<img src="../assets/communication-protocol.png" width="800" />
</div>


The downlink telemetry protocol is designed with a minimal 3-byte header to reduce communication overhead, ensuring the 19.2 kHz air data rate is fully utilized for actual sensor data rather than structural padding.

# FreeRTOS

The drone firmware utilizes FreeRTOS to manage multiple concurrent tasks, ensuring that hard real-time requirements (like attitude control) are never blocked by slower, less critical operations (like saving logs or receiving radio packets).

The system is split into the following primary tasks, arranged by priority:
1.  `controlTask` (Realtime Priority): The absolute core of the drone. It handles all sensor polling (via DMA to prevent CPU blocking), runs the Kalman filters, executes the LQR matrices, and commands the PWM outputs.
    - Execution Rate: The base task is triggered at 100Hz to compute the Attitude inner loops.
    - Velocity Decimation: The middle Velocity PID loop utilizes a `decimation_factor = 5`, meaning it only executes at 20Hz (once every 5 attitude cycles).
    - Position Decimation: The outer Position P loop utilizes a `decimation_factor = 10`, executing at 10Hz (once every 10 attitude cycles). This strict decimation enforces bandwidth separation across the cascaded loops.
2.  `commandTask` (High Priority): Processes incoming LoRa packets from the Ground Station. It updates the global reference targets (Roll, Pitch, Yaw, Z-Altitude) safely via FreeRTOS Mutexes.
3.  `loggerTask` (Above Normal Priority): Packs the state data into the Variable-Field Bitmask format and writes it to the SPI Flash.
4. `telemetryTask` (Normal Priority): Formats basic telemetry packets (battery voltage, altitude, drone state) and queues them for LoRa transmission back to the ground station.
5.  `transferTask` (Realtime Priority - Ground Only): This task remains entirely dormant during flight. It is only activated when the drone receives a "KILL" command. Once active, it monopolizes the CPU to rapidly read the raw SPI Flash memory and write it to the SD Card as a formatted CSV file.

### **Flight State**

The firmware relies on a strict state machine to ensure safety and prevent control windup on the ground:
1. **INIT**: System runs `sys_check()` to test buzzer, servos, motors, and sensors. 
2. **DISARMED**: Drone is idle. Sensors are running, but control loops are bypassed and motors are locked to 0. Integrals are held at zero.
3. **ARMED**: Motors spin up to an idle speed. Control loops engage to hold the current position.
4. **LAUNCH**: The Z-reference target is smoothly ramped up, commanding the drone to ascend.
5. **LAND**: The Z-reference target is slowly ramped down. Once touchdown is detected, the drone automatically disarms.