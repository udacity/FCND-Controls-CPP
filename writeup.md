## Implemented body rate control in C++.

> The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

Body rate control is implemented in `QuadControl::BodyRateControl`.

## Implement roll pitch control in C++.

> The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

This is implemented in `QuadControl::RollPitchControl`. Important thing to note, roll/pitch should be limited to `maxTiltAngle`.
We are actually controlling rotational matrix specific components. And transforming that to roll/pitch in a non linear manner using components of rotational matrix.

## Implement altitude controller in C++.

> The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

> Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

This is implemented in `QuadControl::AltitudeControl`. `integratedAltitudeError` is used to accumulate error. No anti windup is implemented though. Velocity is constrained in region `(-maxAscentRate, maxDescentRate)` as Z points downwards and ascent rate is positive.

## Implement lateral position control in C++.

> The controller should use the local NE position and velocity to generate a commanded local acceleration.

This is implemented in `QuadControl::LateralPositionControl`. Commanded speed and acceleration are limited by vector magnitude.

## Implement yaw control in C++.

> The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

This is done in `QuadControl::YawControl`. `YawError` is used modulo `2*PI`. Then converted to `(-PI,PI)` range.

## Implement calculating the motor commands given commanded thrust and moments in C++.

> The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

This accomplished in `QuadControl::GenerateMotorCommands`. Additional variable `motorsAssign` was used to determine correct order of motors and direction of rotation for each. To solve linear equation matrix inverse was used.