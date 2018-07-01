# The C++ Control Project

A cascaded PID controller has been implemented

## Ruburic Points

### Body Rate Control

A simple propotional controller has been implemented that uses `kpPQR` as the gain variable for minimizing the error between the commanded PQR (attitude of the quad in body frame) and the current attiude.

Returns the moment, which is defined as `M = I * α`, where α is the propotional gain and I is the moment of intertia vector given by `Ixx, Iyy, Izz`

### Roll Pitch Control

A simple propotional controller that uses `kpBank` as the gain variable.

$\dot{b}^x_c  = k_p(b^x_c - b^x_a)$
 


