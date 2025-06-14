#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <iostream>

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

float normalizeAngleDEG(float a);
float normalizeAngleRAD(float a);

char getCharWithoutWaitingENTER(void);

typedef struct{
    float x, y, theta;
} Pose2D;

#endif // UTILS_H
