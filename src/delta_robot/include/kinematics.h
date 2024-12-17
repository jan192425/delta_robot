#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <array>

// Konstanten
#define PI 3.14159265
#define dtr 0.017453293
#define sin30 0.5
#define sin120 0.8660254
#define sinm120 -0.866025
#define cos120 -0.5
#define tan30 0.57735027
#define tan60 1.7320508

// Globale Variablen
extern float re;
extern float rf;
extern float e;
extern float f;
extern float le;
extern float lf;
extern int invout[3][2];

// Funktionen
int fwdkin(float theta1, float theta2, float theta3);
float delta_calcAngleYZ(float xeff, float yeff, float zeff);
int (&invkin(float xeff, float yeff, float zeff))[3][2];

#endif // KINEMATICS_H
