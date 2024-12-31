#include "main.h"
#include <iostream>
#include <fstream>
using namespace std;

//###########################//
//         DEVICE            //
//         CONFIG            //
//###########################//
pros::Motor chassisFR(1);
pros::Motor chassisFL(2);
pros::Motor chassisMR(3);
pros::Motor chassisML(4);
pros::Motor chassisBR(5);
pros::Motor chassisBL(6);

pros::IMU imu(7);



//###########################//
//         DEVICE            //
//         ACTIVITY          //
//         RECORDER          //
//###########################//
double prevChassisRight = 0;
double recordChassisRight(double refreshRate=20) {
	//get ticks per ms of right side
	double currentChassisRight = (chassisFR.get_position() + chassisMR.get_position() + chassisBR.get_position()) / 3;
	double deltaChassisRight = currentChassisRight - prevChassisRight;
	double ticksPerMSRight = deltaChassisRight/refreshRate;

	prevChassisRight = currentChassisRight;
	return ticksPerMSRight;
	pros::delay(refreshRate);
}

double prevChassisLeft = 0;
double recordChassisLeft(double refreshRate=20) {
	//get ticks per ms of left side
	double currentChassisLeft = (chassisFL.get_position() + chassisML.get_position() + chassisBL.get_position()) / 3;
	double deltaChassisLeft = currentChassisLeft - prevChassisLeft;
	double ticksPerMSLeft = deltaChassisLeft/refreshRate;

	prevChassisRight = currentChassisLeft;
	return ticksPerMSLeft;
	pros::delay(refreshRate);
}



//###########################//
//		 WRITING             //
//       TO FILES            //
//###########################//
void writeTelemetryToSD(double durationInSeconds=15, double refreshRate=20) {
	ofstream chassisRightSpeeds("chassisRightSpeeds.txt");
	ofstream chassisLeftSpeeds("chassisLeftSpeeds.txt");
	ofstream IMUValues("IMUValues.txt");

	double startTime = pros::millis();
	while(pros::millis() - startTime < durationInSeconds*1000) {
		chassisRightSpeeds << recordChassisRight(refreshRate) << endl;
		chassisLeftSpeeds << recordChassisLeft(refreshRate) << endl;
		IMUValues << imu.get_heading() << endl;
	}

	chassisRightSpeeds.close();
	chassisLeftSpeeds.close();
	IMUValues.close();
}


//###########################//
//		 READING             //
//       THE FILES           //
//      INTO A VECTOR        //
//###########################//
vector<double> rightSpeeds;
vector<double> leftSpeeds;
vector<double> headings;
void readTelemetryFromSD(double durationInSeconds=15, double refreshRate=20){
	ifstream chassisRightSpeeds("chassisRightSpeeds.txt");
	ifstream chassisLeftSpeeds("chassisLeftSpeeds.txt");
	ifstream IMUValues("IMUValues.txt");

	double time = 0; //used to keep track of how many iterations are needed to get through all the data
	string rightSpeed;
	string leftSpeed;
	string heading;

	while(time < (durationInSeconds*1000)) {
		getline(chassisRightSpeeds, rightSpeed);
		getline(chassisLeftSpeeds, leftSpeed);
		getline(IMUValues, heading);

		//save the values in vectors
		rightSpeeds.push_back(stod(rightSpeed));
		leftSpeeds.push_back(stod(leftSpeed));
		headings.push_back(stod(heading));

		time += refreshRate;
	}

	chassisRightSpeeds.close();
	chassisLeftSpeeds.close();
	IMUValues.close();
}


//###########################//
//       READING THE		 //
//       VECTORS AND		 //
//       MOVING THE ROBOT	 //
//###########################//

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;

		pros::delay(20);
	}
}
