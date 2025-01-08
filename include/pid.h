#ifndef __PID__
#define __PID__

#include "main.h"
#include "global.h"
using namespace glb;

//###########################//
//         PID               //
//###########################//

#define RIGHT_KP 1
#define RIGHT_KI 1
#define RIGHT_KD 1

#define LEFT_KP 1
#define LEFT_KI 1
#define LEFT_KD 1

#define HC_KP 1
#define HC_KI 1
#define HC_KD 1


extern double totalErrorRight = 0; //globalize the integral for the right side so it can carry through multiple time stamps
extern double prevErrorRight; //globalize the derivative for the right side so it can carry through multiple time stamps
extern double calculatePIDRight(double wantedSpeed, double currentSpeed);

extern double totalErrorLeft = 0; //globalize the integral for the left side so it can carry through multiple time stamps
extern double prevErrorLeft; //globalize the derivative for the left side so it can carry through multiple time stamps
extern double calculatePIDLeft(double wantedSpeed, double currentSpeed);

extern double totalErrorHC = 0; //globalize the integral for the heading correction so it can carry through multiple time stamps
extern double prevErrorHC; //globalize the derivative for the heading correction so it can carry through multiple time stamps
extern double calculatePIDHC(double wantedHeading, double currentHeading);


#endif