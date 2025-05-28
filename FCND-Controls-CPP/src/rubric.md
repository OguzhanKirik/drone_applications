## 🚁 QuadControl System Overview

The `QuadControl` class implements a modular, layered control system for a quadrotor drone. It translates high-level trajectory commands into individual motor thrusts, incorporating physics-based models, cascaded control loops, and actuator constraints.

### 🧹 Control Layers & Core Functions

---

### 1. **Body Rate Control**

**Function**: `BodyRateControl(pqrCmd, pqr)`

 Compute required body torques from body rate errors.
* **Controller**: P-controller
* **Equation**: `momentCmd = I * KpPQR * (pqrCmd - pqr)`


  * Uses full moment of inertia scaling via `(Ixx, Iyy, Izz)`
  * Outputs 3D moment vector for roll, pitch, and yaw

---

### 2. **Roll-Pitch Control**

**Function**: `RollPitchControl(accelCmd, attitude, collThrustCmd)`
 Converts desired lateral accelerations into desired roll and pitch rates.


  * Inverts the nonlinear rotation model
  * Constrains tilt angles via `maxTiltAngle`
  * Uses rotation matrix and `kpBank` gain for control

---

### 3. **Altitude Control**

**Function**: `AltitudeControl(posZCmd, velZCmd, posZ, velZ, attitude, accelZCmd, dt)`

* **Purpose**: Converts vertical position/velocity errors into collective thrust.
 PID (P + I + Feedforward)

  * Incorporates `kpPosZ`, `kpVelZ`, and `KiPosZ`
  * Accounts for body orientation using `R(2,2)`
  * Enforces limits via `maxAscentRate` and `maxDescentRate`

---

### 4. **Lateral Position Control**

**Function**: `LateralPositionControl(posCmd, velCmd, pos, vel, accelCmdFF)`

 Computes horizontal acceleration setpoints for lateral motion.
* **Controller**: P-controller + Feedforward


  * Velocity command adjusted from position error
  * Constraints enforced: `maxSpeedXY`, `maxAccelXY`
  * Output acceleration is XY only (`Z = 0`)

---

### 5. **Yaw Control**

**Function**: `YawControl(yawCmd, yaw)`
Converts desired yaw into desired yaw rate.
* **Controller**: P-controller

  * Error wrapped to `[-π, π]` using `fmodf`
  * Gain: `kpYaw`

---

### 6. **Motor Mixing**

**Function**: `GenerateMotorCommands(collThrustCmd, momentCmd)`
 Maps total thrust and torque commands to individual motor thrusts.


  * Uses X-frame geometry (via arm length `L / sqrt(2)`)
  * Includes yaw drag term using `kappa`
  * Constraints: `minMotorThrust`, `maxMotorThrust`

---

### 7. **Integrated Control Loop**

**Function**: `RunControl(dt, simTime)`
 Top-level control sequence integrating all sub-controllers.
* **Sequence**:

  1. Altitude → `collThrustCmd`
  2. Lateral → desired XY acceleration
  3. Roll/Pitch → desired body rates (P, Q)
  4. Yaw → desired yaw rate (R)
  5. Body Rate → desired moments
  6. Motor Mixing → individual thrusts

---




All gains and constraints are loaded from a config system (sim or PX4):
kpPosXY, kpVelXY, kpBank, kpYaw, kpPQR (roll, pitch, yaw rate gains)
maxSpeedXY, maxAccelXY, maxTiltAngle, maxAscentRate, maxDescentRate
minMotorThrust, maxMotorThrust


Scenario 1

Simulation #5 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds

Scenario2
Adjusted kpqr and kpbank.

Simulation #8 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds

Scenario3
Adjusted  parameters kpPosXY and kpPosZ kpVelXY and kpVelZ and 3. component of kpPQR

Simulation #4 (../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds




Scenario 4 
 # Angle control gains
Had to tune kpPosXY

Simulation #3 (../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds

Scenario 5
Simulation #2 (../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds