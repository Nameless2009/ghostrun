#include "main.h"
#include "recorder.h"
using namespace glb;

//###########################//
//         DEVICE            //
//         ACTIVITY          //
//         RECORDER          //
//###########################//
double prevChassisRight = 0;
double recordChassisRight(double refreshRate=20) {
	//get ticks per refresh rate of right side
	double currentChassisRight = (chassisFR.get_position() + chassisMR.get_position() + chassisBR.get_position()) / 3;
	double deltaChassisRight = currentChassisRight - prevChassisRight;
	double ticksPerRRRight = deltaChassisRight/refreshRate;

	prevChassisRight = currentChassisRight;
	return ticksPerRRRight;
	pros::delay(refreshRate);
}

double prevChassisLeft = 0;
double recordChassisLeft(double refreshRate=20) {
	//get ticks per refresh rate of left side
	double currentChassisLeft = (chassisFL.get_position() + chassisML.get_position() + chassisBL.get_position()) / 3;
	double deltaChassisLeft = currentChassisLeft - prevChassisLeft;
	double ticksPerRRLeft = deltaChassisLeft/refreshRate;

	prevChassisLeft = currentChassisLeft; // Corrected line
	return ticksPerRRLeft;
	pros::delay(refreshRate);
}