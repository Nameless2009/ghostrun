#include "main.h"
#include "pid.h"
using namespace glb;

//###########################//
//         PID               //
//###########################//
double totalErrorRight = 0; //globalize the integral for the right side so it can carry through multiple time stamps
double prevErrorRight; //globalize the derivative for the right side so it can carry through multiple time stamps
double calculatePIDRight(double wantedSpeed, double currentSpeed){
	double error = wantedSpeed - currentSpeed;
	// calculate integral
	totalErrorRight += error;

    // calculate derivative
    float derivative = error - prevErrorRight;
    prevErrorRight = error;

    // calculate output
    double speed = (error * RIGHT_KP) + (totalErrorRight * RIGHT_KI) + (derivative * RIGHT_KD);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;
}

double totalErrorLeft = 0; //globalize the integral for the left side so it can carry through multiple time stamps
double prevErrorLeft; //globalize the derivative for the left side so it can carry through multiple time stamps
double calculatePIDLeft(double wantedSpeed, double currentSpeed){
	double error = wantedSpeed - currentSpeed;
	// calculate integral
	totalErrorLeft += error;

	// calculate derivative
	float derivative = error - prevErrorLeft;
	prevErrorLeft = error;

	// calculate output
	double speed = (error * LEFT_KP) + (totalErrorLeft * LEFT_KI) + (derivative * LEFT_KD);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;
}

double totalErrorHC = 0; //globalize the integral for the heading correction so it can carry through multiple time stamps
double prevErrorHC; //globalize the derivative for the heading correction so it can carry through multiple time stamps
double calculatePIDHC(double wantedHeading, double currentHeading){
	//wrap wanted heading
	if (wantedHeading > 180){
		wantedHeading = wantedHeading - 360;
	}

	//wrap current heading
	if (currentHeading > 180){
		currentHeading = currentHeading - 360;
	}

	//turn logic
	if ((wantedHeading < 0) && (currentHeading > 0)){
		if ((currentHeading - wantedHeading) >= 180){
			wantedHeading = wantedHeading + 360;
			currentHeading = imu.get_heading();
		}
	}
	else if ((wantedHeading > 0) && (currentHeading < 0)) {
		if ((wantedHeading - currentHeading) >= 180){
			currentHeading = imu.get_heading();
		}
	}

	//calculate error
	double error = wantedHeading - currentHeading;

	// calculate integral
	totalErrorHC += error;

	// calculate derivative
	float derivative = error - prevErrorHC;
	prevErrorHC = error;

	// calculate output
	double fix = (error * HC_KP) + (totalErrorHC * HC_KI) + (derivative * HC_KD);

	if (fix > 127){
		fix = 127;
	}
	else if (fix < -127){
		fix = -127;
	}

	return fix;
}