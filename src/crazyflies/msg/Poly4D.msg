# One 7th-degree-polynomial piece of a Crazyflie high-level-commander
# trajectory.
#
# value(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5 + c6*t^6 + c7*t^7
#
# t resets to 0 at the start of this piece and runs to `duration` seconds.

float64 duration   # seconds this piece takes to fly

float64[8] x        # x(t) coefficients, c0..c7
float64[8] y        # y(t) coefficients, c0..c7
float64[8] z        # z(t) coefficients, c0..c7
float64[8] yaw      # yaw(t) coefficients, c0..c7