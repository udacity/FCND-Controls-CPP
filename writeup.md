# Project: Building a Controller

## Summary

For this project I implemented a 3D controller for a quadcopter based on a first order Taylor series linearization. This code was based on the Python version built over the course of the previous lesson.

## Rubric points

I addressed the rubric points as follows:

### Altitude control and Motor Commands

I started off by modifying `GenerateMotorCommands` to hard-code a hover and modified the mass configuration paramter until hover was achieved. With that done and having oriented myself to the code, I generalized the thrust commands so that they would be able to respond to any command.

```
float l = L / sqrt(2);
// Calculate force vector
float f_c = collThrustCmd;
float f_p = momentCmd.x / l;        // torque converted to force
float f_q = momentCmd.y / l;        // torque to force
float f_r = momentCmd.z / kappa;    // torque to force
// Calculate individual rotor forces
float f1 = (f_c + f_p + f_q - f_r) / 4.0;
float f2 = f1 - (f_p - f_r) / 2.0;
float f4 = (f_c - f_p) / 2.0 - f2;
float f3 = f_c - f1 - f2 - f4;

cmd.desiredThrustsN[0] = f1;
cmd.desiredThrustsN[1] = f2;
cmd.desiredThrustsN[2] = f3;
cmd.desiredThrustsN[3] = f4;
```


I then built a PID-controller for altitude control. The z-axis velocity error parameter was contstrained to be no more than the maximum ascent rate or less than the maximum descent rate. I also stored the integrated z-axis position error. Finally, this commanded acceleration control was coverted into a force command by differencing it with gravit an multiplying it by the mass of the quad. Finally it was transformed into a body-frame z-axis thrust. 

```
float z_err = posZCmd - posZ;
float z_dot_err = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate) - velZ;
integratedAltitudeError += z_err * dt;
float ubar = kpPosZ * z_err + kpVelZ * z_dot_err + accelZCmd + KiPosZ * integratedAltitudeError;
thrust = -mass * (ubar - CONST_GRAVITY) / R(2, 2);
```

I continued to run scenario one and tuned the parameters until hover was successfully achieved. This set the `mass` paramater at 0.5, `kpPosZ` to 30, `kpVelZ` to 4, and `kiPosZ` to 20.

### Body rate control

To achieve body rate control I used teh `V3F` class to create a moment command. I implemented  a P-controller multiplied the moments of inertia structured in a `V3F` vector. I then tuned `kpPQR` until the tested in scenario two were passed settling on a value of 50, 50, 10.

```
momentCmd = V3F(Ixx, Iyy, Izz) * kpPQR * (pqrCmd - pqr);
```


### Roll and Pitch Control

Roll and pitch control was achieved using a P-controller. I first determined the collective acceleration required from the collective thrust commanded. I then transformed the `x` and `y` accelerations. First these were contrained to be less than the magnitude of the maximum tilt angle. Next the P-controller outputs were calculated in the global frame. Finally, using the attitude Euler-angle rotation matrix, the body frame accelerations were calculated. Scenario 2 was run and `kpBank` was tuned until the roll and pitch scenario was passed settling on a value of 13.

```
pqrCmd.x = 0;
pqrCmd.y = 0;
pqrCmd.z = 0;
if (collThrustCmd > 0)
{
    float c = -collThrustCmd / mass; // convert thrust from Newtons to m/s^2

    float bx = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
    float bx_err = bx - R(0, 2);
    float bx_dot = kpBank * bx_err;

    float by = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
    float by_err = by - R(1, 2);
    float by_dot = kpBank * by_err;

    pqrCmd.x = (R(1, 0) * bx_dot - R(0, 0) * by_dot) / R(2, 2);
    pqrCmd.y = (R(1, 1) * bx_dot - R(0, 1) * by_dot) / R(2, 2);
}
```

### Lateral Control

Lateral position control was achieved using a P-controller to set the three-axis accelerations relative to the current position error and X-Y velocity error. 

```
V3F err = posCmd - pos;
V3F v_err = velCmd - vel;

accelCmd += kpPosXY * err + kpVelXY * v_err;
```

I continued to run scenario 3 and tuned `kpPosXY` and `kpVelXY` until the x- and y-position criteria were passed settling on values of 35 and 12.

### Yaw Control

Finally I implemented a P-controller for yaw and ran scenario 3 until the yaw criteria was passed and by tuning `kpYaw` settling on 5.

```
yawRateCmd = kpYaw * (fmodf(yawCmd, 2 * F_PI) - yaw);
```

## Flight Evaluation

My controller successfully passed all scenarios.