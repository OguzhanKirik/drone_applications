1. **Quaternion-based IMU Integration (UpdateFromIMU)**
   - Created a quaternion from the current roll, pitch, and yaw estimates using `FromEuler123_RPY`.
   - Integrated the gyroscope body rates into the quaternion using `IntegrateBodyRate`.
   - Extracted updated roll, pitch, and yaw values from the quaternion.
   - Normalized the yaw to stay within [-π, π].
   - Updated the EKF state yaw component with the normalized yaw value.

2. **State Prediction Using Accelerometer Data (PredictState)**
   - Predicted position components (x, y, z) by integrating velocity using simple Euler integration.
   - Created a gravity vector to subtract gravity from the inertial frame acceleration.
   - Rotated body-frame acceleration into inertial frame using the estimated attitude.
   - Predicted velocity components (vx, vy, vz) using inertial frame acceleration and time step.

3. **EKF Covariance Prediction (Predict)**
   - Set Jacobian elements for position update using velocity: `gPrime(i, i+3) = dt`.
   - Computed derivatives of velocity with respect to yaw using rows of `RbgPrime` dot `accel`.
   - Updated the Jacobian matrix `gPrime` accordingly.
   - Applied the EKF covariance prediction formula: `ekfCov = gPrime * ekfCov * gPrime.transpose() + Q`.

4. **GPS Update (UpdateFromGPS)**
   - Populated the measurement vector `zFromX` with state values directly (position and velocity).
   - Set diagonal values of the measurement Jacobian `hPrime` to 1 for direct state observations.

5. **Magnetometer Yaw Update (UpdateFromMag)**
   - Extracted yaw from the state as measurement prediction.
   - Normalized the yaw difference `z - zFromX` to avoid wrap-around discontinuities.
   - Updated `zFromX` accordingly to keep yaw error within [-π, π].
   - Set the appropriate element in the Jacobian matrix to indicate observation of yaw only.

**Scenario 6: Sensor Noise (Simulation #2 - ../config/06_SensorNoise.txt)**
- ✅ PASS: Absolute GPS position error (X) was within MeasuredStdDev_GPSPosXY for 68% of the time.
- ✅ PASS: Absolute IMU acceleration error (AX) was within MeasuredStdDev_AccelXY for 67% of the time.

**Scenario 7: Attitude Estimation (Simulation #3 - ../config/07_AttitudeEstimation.txt)**
- ✅ PASS: Maximum Euler angle error was less than 0.1 radians for at least 3.0 seconds.

**Scenario 8: Predict State (Simulation #2 - ../config/08_PredictState.txt)**
- ℹ️ No criteria specified for this scenario.

**Scenario 9: Predict Covariance (Simulation #9 - ../config/09_PredictCovariance.txt)**
- ℹ️ No criteria specified for this scenario.

**Scenario 10: Magnetometer Update (Simulation #2 - ../config/10_MagUpdate.txt)**
- ✅ PASS: Absolute yaw error was less than 0.12 radians for at least 10.0 seconds.
- ✅ PASS: Absolute yaw innovation was less than its estimated standard deviation for 79% of the time.

**Scenario 11: GPS Update with New Control Parameters (Simulation #2 - ../config/11_GPSUpdate.txt)**
- ✅ PASS: Absolute position error was less than 1.0 meter for at least 20.0 seconds.

