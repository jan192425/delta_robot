#include <iostream>
#include <math.h>
#include <array>
//? based on: https://hypertriangle.com/~alex/delta-robot-tutorial/  

#define PI  3.14159265
#define dtr  0.017453293  //from deg to rad
#define sin30  0.5
#define sin120  0.8660254
#define sinm120 -0.866025
#define cos120  -0.5      //= cos(-120)
#define tan30  0.57735027
#define tan60  1.7320508

using namespace std;

//! Radii must be set according to the position of the intersection of the arm's symmetry axis
//! with the axis of either the motorshaft or the endeffector joint shaft !!!!!!!!!!!!!!!!!!!!!!
float re = 30;  //effector radius in mm
float rf = 100; //base radius in mm
float e = 2*re*tan60;  //length of triangle side of endeffector
float f = 2*rf*tan60;  //length of triangle side of base

float le = 300; //effector arm ("forearm") lenght in mm
float lf = 200; //base arm ("biceps") lenght in mm

int invout [3][2] = {{1,5},{2,5},{3,5}};

//?------forward kinematics-------
//? input the current angles theta of the motors 1,2,3 and return the current position of the endeffector (poseff)
int fwdkin (float theta1, float theta2, float theta3) {
    float t = (f-e)*tan30/2; //Delta of the distance from the base (f) and effector (e) sides to z axis (common centerpoint of the triangles in start position)
    
    theta1 *= dtr;   
    theta2 *= dtr;
    theta2 *= dtr;

    //Calculation of the joint positions with joint1 in the yz-plane and the other arms 30° rotated to relative to the x-axis
    float y1 = -(t + rf*cos(theta1));
    float z1 = -rf*sin(theta1);
    
    float y2 = (t + rf*cos(theta2))*sin30;
    float x2 = y2*tan60;
    float z2 = -rf*sin(theta2);
    
    float y3 = (t + rf*cos(theta3))*sin30;
    float x3 = -y3*tan60;
    float z3 = -rf*sin(theta3);
    
    //*One spheres around each joint with radius re  >>> (sphere1-sphere2) & (sphere1-sphere3) & (sphere2-sphere3) 
    //* >>> isolate y and x in (s2-s3) seperately >>> substitute y in (s1-32) to get x (I) & substitute x in (s1-s3) to get y (II)
    float w1 = y1*y1 + z1*z1;            //extra variables w to short the following equations
    float w2 = x2*x2 + y2*y2 + z2*z2;
    float w3 = x3*x3 + y3*y3 + z3*z3;

    float dnm = (y2-y1)*x3-(y3-y1)*x2;

    //* (I) x = (a1*z + b1)/dnm
    float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
    //* (II) y = (a2*z + b2)/dnm;
    float a2 = -(z2-z1)*x3+(z3-z1)*x2;
    float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
    //* Substitute y and x in the equation for sphere1 and solve the resulting quadratic equation to get z
    float a = a1*a1 + a2*a2 + dnm*dnm;
    float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
    // discriminant
    float d = b*b - (float)4.0*a*c;
    if (d < 0) return -1; // non-existing point

    //resulting coords of the effector
    float zeff = -(float)0.5*(b+sqrt(d))/a;
    float xeff = (a1*zeff + b1)/dnm;
    float yeff = (a2*zeff + b2)/dnm;

    
    float poseff[3] = {xeff,yeff,zeff};

    return 0;
}


//helper function for calculating the motorangles theta (by reference)
float delta_calcAngleYZ(float xeff, float yeff, float zeff) {  
     float y1 = -0.5* tan30*f;  // get coords of motorshaft1 which lies in the yz-plane
     xeff -= 0.5*tan30*e;         // shift effector center to edge perpendicular to yz-plane

     // z = a + b*y | linear equation via subtraction of the 2 circle equations and solve after z
     float a = (xeff*xeff + yeff*yeff + zeff*zeff +lf*lf - le*le - y1*y1)/(2*zeff);
     float b = (y1-yeff)/zeff;

     //substitute the lin. eq. back into one of the 2 circle equations and compute the discriminant
     float d = -((a+b*y1)*(a+b*y1))+lf*(b*b*lf+lf); 
     //if (d < 0) return 0; //! non-reachable point with the given kinematics => then return biceps zu origin

     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point (= - branch of the quadratic equation)
     float zj = a + b*yj;
     float theta = 180.0*atan(-zj/(y1 - yj))/PI; //+ ((yj>y1)?180.0:0.0);    
     return theta;
 }

//?------inverse kinematics-------
//? input the goal endeffector position (poseff) and return goal angles theta of the motors 1,2,3 
//? function referencing the array invout to be able to return invout afterwards
int (&invkin(float xeff, float yeff, float zeff))[3][2]{
     float theta1 = delta_calcAngleYZ(xeff, yeff, zeff);
     float theta2 = delta_calcAngleYZ(xeff*cos120 + yeff*sin120, yeff*cos120-xeff*sin120, zeff);  // rotate coords to +120 deg via multiplication with (standard) rotation matrix
     float theta3 = delta_calcAngleYZ(xeff*cos120 - yeff*sin120, yeff*cos120+xeff*sin120, zeff);  // rotate coords to -120 deg via multiplication with(standard) rotation matrix
     /*
     theta1 = ((theta1/dtr)/0.088);
     theta2 = ((theta2/dtr)/0.088);
     theta3 = ((theta3/dtr)/0.088);
        */
     theta1 = (theta1/0.088); //! NEGATIVE WERTE VERMEIDEN => KANN POS CONTROL MODE NICHT VERWERTEN
     theta2 = (theta2/0.088);
     theta3 = (theta3/0.088);

     int posmotor1 = round(theta1);
     int posmotor2 = round(theta2);
     int posmotor3 = round(theta3);

     invout[0][1] = posmotor1;
     invout[1][1] = posmotor2;
     invout[2][1] = posmotor3;

     return invout;
}