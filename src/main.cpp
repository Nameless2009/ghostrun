#include "global.cpp"
#include "main.h"
#include "api.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdio>
#include "unistd.h"

using namespace std;
using namespace global;

double timestamp;                           // Timestamp             //
bool recorded;	                            // Recorded?             //

std::vector<double> Voltage_L = { };        // Left Side of Chassis  // 
std::vector<double> Voltage_R = { };        // Right Side of Chassis //
std::vector<double> Voltage_LB = { };       // Lady Brown            //
std::vector<double> Voltage_In = { };       // Intake 	             //
std::vector<double> Voltage_C = { };        // Clamp	             //
std::vector<double> Voltage_LB_time = { };  // Lady Brown Timestamp  //
std::vector<double> Voltage_In_time = { };  // Intake Timestamp      //
std::vector<double> Voltage_C_time = { };   // Clamp Timestamp 	     //

void Initialization( ){
	global::RC.move_velocity(100);
	global::LC.move_velocity(100);
	global::LadyBrown.move_velocity(100);
	global::Intake.move_velocity(100);
	global::RC.move_voltage(0);
	global::LC.move_voltage(0);
	global::LadyBrown.move_voltage(0);
	global::Intake.move_voltage(0);}

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
		timestamp = pros::millis();
		global::LadyBrown.move_voltage(127);
		Voltage_LB.push_back(127);}
		Voltage_LB_time.push_back(timestamp);
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_R1)){
		timestamp = pros::millis();
		global::LadyBrown.move_voltage(-127);
		Voltage_In.push_back(-127);
		Voltage_LB_time.push_back(timestamp);}
	else {
		Voltage_In.push_back(0);}}

void Intake_Control( ) {
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_L2)){
		timestamp = pros::millis();
		global::Intake.move(127);
		Voltage_In.push_back(127);
		Voltage_In_time.push_back(timestamp);}
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_R2)){
		timestamp = pros::millis();
		global::Intake.move(-127);
		Voltage_In.push_back(127);
		Voltage_In_time.push_back(timestamp);}}

void Clamp_Control( ){
	timestamp = pros::millis();
	//Voltage_C.push_back(127);
	Voltage_C_time.push_back(timestamp);
	//REMEBER TO DO THIS BY 1/7/25
}

bool Volatage_Record(){
	if (global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_L2) && global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_R2) && global::DriversInput.get_digital(E_CONTROLLER_DIGITAL_A)){
		FILE* Voltage_L_file = fopen("/usd/Voltag_L.bin", "wb");
		FILE* Voltage_R_file = fopen("/usd/Voltage_R.bin","wb");
		FILE* Voltage_LB_file = fopen("/usd/Voltage_LB.bin", "wb");
		FILE* Voltage_In_file = fopen("/usd/Voltage_In.bin", "wb");
		FILE* Voltage_C_file = fopen("/usd/Voltage_C.bin","wb");
		FILE* Voltage_LB_time_file = fopen("/usd/Voltage_LB_time.bin","wb");
		FILE* Voltage_In_time_file = fopen("/usd/Voltage_In_time.bin","wb");
		FILE* Voltage_C_time_file = fopen("/usd/Voltage_C_time.bin","wb");

		if (!Voltage_L_file || !Voltage_R_file || !Voltage_LB_file || !Voltage_In_file || !Voltage_C_file || !Voltage_LB_time_file || !Voltage_In_time_file || !Voltage_C_time_file) {
			DriversInput.clear();
			DriversInput.print(0,1, "Error 1: Failed to open file.");
		return;}

		DriversInput.clear();
		DriversInput.print(0,1, "Message 4: Files opened.");
		DriversInput.print(0,2, "Message 1: Recording drive.");

		int8_t Voltage_L;
		int8_t Voltage_R;
		int8_t Voltage_LB; 
		int8_t Voltage_In;
		int8_t Voltage_C;
		int8_t Voltage_LB_time;
		int8_t Voltage_In_time;
		int8_t Voltage_C_time;

		fwrite(&Voltage_L , sizeof(Voltage_L) , 1, Voltage_L_file );
		fwrite(&Voltage_R , sizeof(Voltage_R) , 1, Voltage_R_file );
		fwrite(&Voltage_LB, sizeof(Voltage_LB), 1, Voltage_LB_file);
		fwrite(&Voltage_In, sizeof(Voltage_In), 1, Voltage_In_file);
		fwrite(&Voltage_LB_time, sizeof(Voltage_LB_time), 1, Voltage_LB_time_file);
		fwrite(&Voltage_In_time, sizeof(Voltage_In_time), 1, Voltage_In_time_file);
		fwrite(&Voltage_C_time, sizeof(Voltage_C_time), 1, Voltage_C_time_file);
		DriversInput.print(0,3, "Message 2: Drive recorded.");

		fclose(Voltage_L_file);
		fclose(Voltage_R_file);
		fclose(Voltage_LB_file);
		fclose(Voltage_In_file);
		fclose(Voltage_C_file);
		fclose(Voltage_LB_time_file);
		fclose(Voltage_In_time_file);
		fclose(Voltage_C_time_file);
		recorded=true;
		DriversInput.print(0,4, "Message 3: Recording files closed.");

		return recorded;
		}
	else{};
	
}

std::vector<double> Voltage_L_read;
std::vector<double> Voltag_R_read;

void Voltage_Playback_Data_Chassis(){
	FILE* Voltage_L_file = fopen("/usd/Voltag_L.bin", "rb");
	FILE* Voltage_R_file = fopen("/usd/Voltage_R.bin","rb");	

	if (!Voltage_L_file || !Voltage_R_file) {
		DriversInput.clear();
		DriversInput.print(0,1, "Error 2: Failed to locate and read files.");	
		return;	
	}

	//Do I add while loop? Ask Ayush about his

	int8_t Voltage_L;
	int8_t Voltage_R;
	
	size_t rsRead = fread(&Voltage_L, sizeof(Voltage_L), 1, Voltage_L_file);
	size_t lsRead = fread(&Voltage_R, sizeof(Voltage_R), 1, Voltage_R_file);
	
	if (rsRead < 1 || lsRead < 1 ) {
		DriversInput.clear();
		DriversInput.print(0,1, "Error 3: Less than one byte was read.");
		return;
	}

	Voltage_L_read.push_back(static_cast<double>(Voltage_L));
	Voltage_L_read.push_back(static_cast<double>(Voltage_R));

	fclose(Voltage_L_file);
	fclose(Voltage_R_file);

}

void Voltage_Playback_Data_Timestamps(){

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
		Volatage_Record();
		if (recorded == true){
			break;
		}
	}
}
