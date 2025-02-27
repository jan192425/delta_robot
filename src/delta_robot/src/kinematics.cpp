#include <iostream>
#include <math.h>
#include <array>
#include <stdio.h>
#include <cmath>
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
float re = 30.0;  //effector radius in mm
float rf = 95.0; //base radius in mm
float e = 2*re*tan60;  //length of triangle side of endeffector
float f = 2*rf*tan60;  //length of triangle side of base
float deg2pulse = 4096.0 / 360.0;

float le = 300.0; //effector arm ("forearm") lenght in mm
float lf = 200.0; //base arm ("biceps") lenght in mm

int invout [3][3] = {{1,5,0},{2,5,0},{3,5,0}};
float prevangle [3] = {0,0,0}; //array that stores the theta angles of the previous inverse calculation 
std::array<float, 3> poseff = {0,0,0};
std::array<float, 3> prevposition = {1,0,5}; //array that stores the theta angles of the previous forward calculation 
int i = 0; 

float minangle = -30.0;
float maxangle = 90.0;

//?------forward kinematics-------
//? input the current angles theta of the motors 1,2,3 and return the current position of the endeffector (poseff)
std::array<float, 3> fwdkin(int motpos1, int motpos2, int motpos3) {

    float t = (f-e)*tan30/2; //Delta of the distance from the base (f) and effector (e) sides to z axis (common centerpoint of the triangles in start position)
    
    float theta1 = -dtr*((motpos1*(1/deg2pulse))-270);  
    float theta2 = -dtr*((motpos2*(1/deg2pulse))-270);
    float theta3 = -dtr*((motpos3*(1/deg2pulse))-270);      

    //Calculation of the joint positions with joint1 in the yz-plane and the other arms 30° rotated to relative to the x-axis
    float  y1 = -(t + lf*cos(theta1));
    float z1 = -lf*sin(theta1);
    
    float y2 = (t + lf*cos(theta2))*sin30;
    float  x2 = y2*tan60;
    float z2 = -lf*sin(theta2);
    
    float y3 = (t + lf*cos(theta3))*sin30;
    float x3 = -y3*tan60;
    float z3 = -lf*sin(theta3);
    
    //*One spheres around each joint with radius re  >>> (sphere1-sphere2) & (sphere1-sphere3) & (sphere2-sphere3) 
    //* >>> isolate y and x in (s2-s3) seperately >>> substitute y in (s1-32) to get x (I) & substitute x in (s1-s3) to get y (II)
    float w1 = y1*y1 + z1*z1;            //extra variables w to short the following equations
    float w2 = x2*x2 + y2*y2 + z2*z2;
    float w3 = x3*x3 + y3*y3 + z3*z3;

    float dnm = (y2-y1)*x3-(y3-y1)*x2;

    //* (I) x = (a1*z + b1)/dnm
    float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/(float)2.0;
 
    //* (II) y = (a2*z + b2)/dnm;
    float a2 = -(z2-z1)*x3+(z3-z1)*x2;
    float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/(float)2.0;
 
    //* Substitute y and x in the equation for sphere1 and solve the resulting quadratic equation to get z
    float a = a1*a1 + a2*a2 + dnm*dnm;
    float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - le*le); //! - gehört da laut code eigentlich nicht hin => aber so funktionierts(optisc) tatsächlich UND VERÄNDERT Z  TWIST ZU FALSCHEN WERTEN
  
    // discriminant
    float d = b*b - (float)4.0*a*c;
    if (d < 0) return {a,b,c}; // non-existing point
   // In der Funktion fwdkin():
   
    //resulting coords of the effector
    float zeff = -0.5*(b+sqrt(d))/a;
    float xeff = (a1*zeff + b1)/dnm;
    float yeff = (a2*zeff + b2)/dnm;
   
    prevposition [0] = theta1;
    prevposition [1] = theta2;
    prevposition [2] = -zeff;
    
   
   if (abs(xeff) < 50){
      xeff = 0;
   }
   if (abs(yeff) < 50){
      yeff = 0;
   }
   
    poseff[0] = xeff;
    poseff[1] = yeff;
    poseff[2] = -zeff;
 
    return poseff;
}


//helper function for calculating the motorangles theta (by reference)
float delta_calcAngleYZ(float xeff, float yeff, float zeff, int i) { 
     
     float y1 = -0.5* tan30*f;  // get coords of motorshaft1 which lies in the yz-plane
     yeff -= 0.5*tan30*e;         // shift effector center to edge perpendicular to yz-plane

     // z = a + b*y | linear equation via subtraction of the 2 circle equations and solve after z
     float a = (xeff*xeff + yeff*yeff + zeff*zeff +lf*lf - le*le - y1*y1)/(2*zeff);
     float b = (y1-yeff)/zeff;

     //substitute the lin. eq. back into one of the 2 circle equations and compute the discriminant
     float d = -((a+b*y1)*(a+b*y1))+lf*(b*b*lf+lf); 
     if (d < 0) return prevangle[i]; //! if non-reachable point with the given kinematics => then biceps will stay in previous position

     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point (= - branch of the quadratic equation)
     float zj = a + b*yj;
     float theta = 180.0*atan(-zj/(y1 - yj))/PI; //Output in Degree
     if (theta < -30 || theta > 90)
        return prevangle[i];
     else 
        return theta;
 }

//?------inverse kinematics-------
//? input the goal endeffector position (poseff) and return goal angles theta of the motors 1,2,3 
//? function referencing the array invout to be able to return invout afterwards
int (&invkin(float xeff, float yeff, float zeff))[3][3]{
     i = 0;   
     float theta1 = delta_calcAngleYZ(xeff, yeff, zeff, i);
     i++;
     float theta2 = delta_calcAngleYZ(xeff*cos120 + yeff*sin120, yeff*cos120-xeff*sin120, zeff, i);  // rotate coords to +120 deg via multiplication with (standard) rotation matrix
     i++;
     float theta3 = delta_calcAngleYZ(xeff*cos120 - yeff*sin120, yeff*cos120+xeff*sin120, zeff, i);  // rotate coords to -120 deg via multiplication with(standard) rotation matrix

     prevangle [0] = theta1;
     prevangle [1] = theta2;
     prevangle [2] = theta3;

     /*
     theta1 = ((theta1/dtr)/0.088);
     theta2 = ((theta2/dtr)/0.088);
     theta3 = ((theta3/dtr)/0.088);
        */

       
     float angle1 = ((-theta1+270.0)*deg2pulse); //+270 so that negative motorpoitions (0...4095) are avoided
     float angle2 = ((-theta2+270.0)*deg2pulse); 
     float angle3 = ((-theta3+270.0)*deg2pulse);

     int posmotor1 = static_cast<int>(round(angle1));
     int posmotor2 = static_cast<int>(round(angle2));
     int posmotor3 = static_cast<int>(round(angle3));

    //! max Winkel 300 => 3409

     //int posmotor2 = (int)angle2;
     //int posmotor3 = (int)angle3;

     invout[0][1] = posmotor1;
     invout[1][1] = posmotor2;
     invout[2][1] = posmotor3;
     invout[0][2] = theta1;
     invout[1][2] = theta2;
     invout[2][2] = theta3;

     return invout;
}