#include "main.h"
#include "drivercontrol.h"
using namespace glb;

//###########################//
//     DRIVER CONTROLS       //
//###########################//
void driverControl(){
	//get joystick values
	int leftJoystick = con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int rightJoystick = con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	//run the motors
	chassisFR.move(rightJoystick);
	chassisMR.move(rightJoystick);
	chassisBR.move(rightJoystick);

	chassisFL.move(leftJoystick);
	chassisML.move(leftJoystick);
	chassisBL.move(leftJoystick);
}