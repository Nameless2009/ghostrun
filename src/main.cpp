#include "global.cpp"
#include "main.h"
#include "api.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdio>
#include "unistd.h"

using namespace std; using namespace global;

double timestamp;                          // Timestamp //
bool recorded;	                           // Recorded? //

std::vector<double> Voltage_L = { };       // Left Side of Chassis  // 
std::vector<double> Voltage_R = { };       // Right Side of Chassis //
std::vector<double> Voltage_LB = { };      // Lady Brown            //
std::vector<double> Voltage_In = { };      // Intake 	            //
std::vector<double> Voltage_C = { };       // Clamp				    //
std::vector<double> Voltage_LB_time = { }; // Lady Brown Timestamp 	//
std::vector<double> Voltage_In_time = { }; // Intake Timestamp      //
std::vector<double> Voltage_C_time = { };  // Clamp Timestamp 		//

void Program_Initialization( ){ global::RC.move_velocity(100); global::LC.move_velocity(100); global::LadyBrown.move_velocity(100); global::Intake.move_velocity(100); global::RC.move_voltage(0); global::LC.move_voltage(0); global::LadyBrown.move_voltage(0); global::Intake.move_voltage(0);}

double Voltage_Adjuster(double Input_Voltage, double Output_Voltage, double Motor_Efficiency, double Motor_Heat, double M_constant ){
	double needed_voltage = Input_Voltage * Motor_Efficiency * Motor_Heat * M_constant;//Relationshp between all things
	if (  needed_voltage > 127) {
		return needed_voltage = 127;}
	if ( needed_voltage < -127) {
		return needed_voltage = -127;}
	else{
		return needed_voltage;}}

void l(double lv){
	global::L1.move_voltage(-lv);
	global::L2.move_voltage(-lv);
	global::L3.move_voltage(lv);
}
void r(double rv){
	global::R1.move_voltage(-rv);
	global::R2.move_voltage(-rv);
	global::R3.move_voltage(rv);
}

std::vector<double> Voltage_L_read;
std::vector<double> Voltage_R_read;
std::vector<double> Voltage_LB_read;
std::vector<double> Voltage_In_read;
std::vector<double> Voltage_C_read;
std::vector<double> Voltage_LB_time_read;
std::vector<double> Voltage_In_time_read;
std::vector<double> Voltage_C_time_read;

double current_voltage; double launch_time; double current_time;

void Voltage_Playback_Data(){
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
			DriversInput.print(0,1, "Error 2: Failed to open files.");
		return;}
	//Do I add while loop? Ask Ayush about his
	int8_t Voltage_L; int8_t Voltage_R; int8_t Voltage_LB; int8_t Voltage_In; int8_t Voltage_C; int8_t Voltage_LB_time; int8_t Voltage_In_time; int8_t Voltage_C_time;
	size_t R = fread(&Voltage_L, sizeof(Voltage_L), 1, Voltage_L_file);
	size_t L = fread(&Voltage_R, sizeof(Voltage_R), 1, Voltage_R_file);
	size_t LB = fread(&Voltage_LB, sizeof(Voltage_LB), 1, Voltage_LB_file);
	size_t In = fread(&Voltage_In, sizeof(Voltage_In), 1, Voltage_In_file);
	size_t C = fread(&Voltage_C, sizeof(Voltage_C), 1, Voltage_C_file);
	size_t LB_t = fread(&Voltage_LB_time, sizeof(Voltage_LB_time), 1, Voltage_LB_time_file);
	size_t In_t = fread(&Voltage_In_time, sizeof(Voltage_In_time), 1, Voltage_In_time_file);
	size_t C_t = fread(&Voltage_C_time, sizeof(Voltage_C_time), 1, Voltage_C_time_file);
	if (R < 1 || L < 1 || LB < 1 || In < 1 || C < 1 || LB_t < 1 || In_t < 1 || C_t < 1) {
		DriversInput.clear();
		DriversInput.print(0,1, "Error 3: Less than one byte was read.");
		return;}
	Voltage_L_read.push_back(static_cast<double>(Voltage_L));
	Voltage_L_read.push_back(static_cast<double>(Voltage_R));
	Voltage_LB_read.push_back(static_cast<double>(Voltage_LB));
	Voltage_In_read.push_back(static_cast<double>(Voltage_In));
	Voltage_C_read.push_back(static_cast<double>(Voltage_C));
	Voltage_LB_time_read.push_back(static_cast<double>(Voltage_LB_time));
	Voltage_In_time_read.push_back(static_cast<double>(Voltage_In_time));
	Voltage_C_time_read.push_back(static_cast<double>(Voltage_C_time));
	fclose(Voltage_L_file);
	fclose(Voltage_R_file);
	fclose(Voltage_LB_file);
	fclose(Voltage_In_file);
	fclose(Voltage_C_file);
	fclose(Voltage_LB_time_file);
	fclose(Voltage_In_time_file);
	fclose(Voltage_C_time_file);}

void Chassis_Playback(){
	if (Voltage_L_read.empty()){}
	else{
		current_voltage = Voltage_L_read.front();
		l(current_voltage);
		Voltage_L_read.erase(Voltage_L_read.begin());
		current_voltage = Voltage_R_read.front();
		r(current_voltage);
		Voltage_R_read.erase(Voltage_R.begin());}}

void Lady_Brown_Playback(){
	if (Voltage_LB_read.empty()){}
	else{
		launch_time = Voltage_LB_time_read.front();
		current_time = pros::millis();
		if ( current_time = launch_time ){
			current_voltage = Voltage_LB_read.front();
			Voltage_LB_read.erase(Voltage_LB_read.begin());
		}
	else{};}}

void Intake_Playlist(){
	if (Voltage_In_read.empty()){}
	else{
		launch_time = Voltage_In_time_read.front();
		current_time = pros::millis();
		if ( current_time = launch_time ){
			current_voltage = Voltage_In_read.front();
			Voltage_In_read.erase(Voltage_In_read.begin());}
		else{}}}

void Clamp_Playback(){
	if (Voltage_C_read.empty()){}
	else{
		launch_time = Voltage_C_time_read.front();
		current_time = pros::millis();
		if ( current_time = launch_time ){
			current_voltage = Voltage_C_read.front();
			Voltage_C_read.erase(Voltage_C_read.begin());}
		else{};}}
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
void on_center_button() {static bool pressed = false; pressed = !pressed; if (pressed) {pros::lcd::set_text(2, "I was pressed!");} else {pros::lcd::clear_line(2);}}
 

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {pros::lcd::initialize();pros::lcd::set_text(1, "Hello PROS User!");pros::lcd::register_btn1_cb(on_center_button);}

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
void autonomous() {Chassis_Playback();Lady_Brown_Playback();Intake_Playlist();Clamp_Playback();if (millis() == (15*1000)){/*End this shit*/}
}

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
	Program_Initialization();
	Voltage_Playback_Data();
	Chassis_Playback();
	Intake_Playlist();
	
	while (true) {

	}
}
