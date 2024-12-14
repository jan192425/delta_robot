#ifndef KINEMATICS_H
#define KINEMATICS_H


float fwdkin (float theta1, float theta2, float theta3);
int invkin (float xeff, float yeff, float zeff, float &theta1, float &theta2, float &theta3);

float poseff[3];

#endif