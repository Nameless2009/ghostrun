#include "global.cpp"
#include "main.h"
#include "api.h"
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <cstdio>

using namespace std;
using namespace global;

std::list<int> Voltage_L = { };  // Left Side of Chassis  // 
std::list<int> Voltage_R = { };  // Right Side of Chassis //
std::list<int> Voltage_LB = { }; // Lady Brown            //
std::list<int> Voltage_In = { }; // Intake 	              //

void Initialization( ){
	global::RC.move_velocity(100);
	global::LC.move_velocity(100);
	global::LadyBrown.move_velocity(100);
	global::Intake.move_velocity(100);
	global::RC.move_voltage(100);
	global::LC.move_voltage(100);
	global::LadyBrown.move_voltage(100);
	global::Intake.move_voltage(100);}

int Chassis_Control( ) {
	double leftstick = global::DriversInput.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
	double rightstick = global::DriversInput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

	global::RC.move_voltage(rightstick);
	global::LC.move_voltage(rightstick);
	Voltage_R.push_back(leftstick);
	Voltage_L.push_back(rightstick);
	if (leftstick >= 0){
		global::RC.move_voltage(leftstick);
		global::LC.move_voltage(leftstick * -1 );}
	if (leftstick < 0 ){
		global::LC.move_voltage(leftstick);
		global::RC.move_voltage(leftstick * -1 );}}


void Lady_Brown_Control( ) {
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_L1)){
		global::LadyBrown.move_voltage(127);
		Voltage_LB.push_back(127);}
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_R1)){
		global::LadyBrown.move_voltage(-127);
		Voltage_In.push_back(-127);}
	else {
		Voltage_In.push_back(0);}}

//Intake///
void Intake_Control( ) {
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_L2)){
		global::Intake.move(127);
		Voltage_In.push_back(127);}
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_R2)){
		global::Intake.move(-127);
		Voltage_In.push_back(127);}	
	else {
		Voltage_In.push_back(0);}}

void Clamp_Control( ){
	//REMEBER TO DO THIS BY 1/7/25
}

void Volatage_Record(){
	//Will include in later push. I dont want everything crashing cause I messed with file printing.
}

//PID SYSTEM//
int PID_S(int target,int kp,int ki,int kd) {
	
}

// int PID_T(target) {
// }

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}
//COMMENTS MADE BY NATHAN//
//USE ALL IMPORTED AUTON FUNCTIONS//

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	while (true) {
		Initialization();
		Chassis_Control();
		Intake_Control();
		Lady_Brown_Control();
		Clamp_Control();
		Volatage_Record():
	}

}
