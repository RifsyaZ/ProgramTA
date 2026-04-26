(MATH Drive Kinematics)
radius : 800

omega : 7.4

mapfloat (omegaL/R - 0) * (PWM MAX - PWM MIN) / (PWM MAX - 0) + 20

v_right : speed + (omega * base /2)

omega right : 100.999
omega left  : 99.001

Note:
1. gerak putar : turnn(speed 20 - 35, 10/-10);
2. maju mundur : wheel(speed, sudutZX);
3. radius : turnn(speed, 200 - 1000);
4. geser : turnn(speed, 0, literal);