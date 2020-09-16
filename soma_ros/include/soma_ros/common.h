#ifndef COMMON_H
#define COMMON_H

#include <math.h>

#define WHEEL_BASE 1.4

#define RAD2DEG(x) (x / M_PI * 180.0)
#define DEG2RAD(x) (x / 180.0 * M_PI)

// #define Float2(x) ((float)((int)(x*100.0))/100.0)
#define POW2(x) pow(x, 2.0)

static inline float Float2(float x)
{
    return (float)((int)(x * 100.0)) / 100.0;
}

static double Dist(double x1, double x2, double y1, double y2)
{
    double d = 0.0;
    double dx = x2 - x1;
    double dy = y2 - y1;
    d = sqrt(POW2(dx) + POW2(dy));
    return d;
}

//car-like motion model t-1 to t
static void motion_model(double _x, double _y, double _theta,
                        double _lambda, double _v, double dt,
                        double &x, double &y, double &theta)
{
    if(_lambda == 0.0 && _v != 0.0){ //straight motion
        x = _x - _v*dt*sin(_theta);
        y = _y + _v*dt*cos(_theta);
        theta = _theta;
        return;
    }

    if(_v == 0.0){
        x = _x;
        y = _y;
        theta = _theta;
        return;
    }

    //estimate yaw late [lad/sec]
    double omega = _v*sin(_lambda)/(double)(WHEEL_BASE);

    x = _x - _v/omega*sin(_theta) + _v/omega*sin(_theta + omega*dt);
    y = _y + _v/omega*cos(_theta) - _v/omega*cos(_theta + omega*dt);
    theta = _theta + omega*dt;
}

#endif