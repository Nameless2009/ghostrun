#include "main.h"
// #include <iostream>
// #include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include "unistd.h"
using namespace std;

#define RIGHT_KP 1
#define RIGHT_KI 1
#define RIGHT_KD 1

#define LEFT_KP 1
#define LEFT_KI 1
#define LEFT_KD 1

#define HC_KP 1
#define HC_KI 1
#define HC_KD 1


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

pros::Controller con(pros::E_CONTROLLER_MASTER);



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



//###########################//
//		 WRITING             //
//       TO FILES            //
//###########################//
void writeToSD(double durationInSeconds=15, double refreshRate=20){
	// ofstream chassisRightSpeeds("chassisRightSpeeds.bin", ios::binary);
	// ofstream chassisLeftSpeeds("chassisLeftSpeeds.bin", ios::binary);
	// ofstream IMUValues("IMUValues.bin", ios::binary);
	FILE* chassisRightSpeeds = fopen("/usd/chassisRightSpeeds.bin", "wb");
	FILE* chassisLeftSpeeds = fopen("/usd/chassisLeftSpeeds.bin", "wb");
	FILE* IMUValues = fopen("/usd/IMUValues.bin", "wb");
	FILE* chassisRightPos = fopen("/usd/chassisRightPos.bin", "wb");
	FILE* chassisLeftPos = fopen("/usd/chassisLeftPos.bin", "wb");

	//catching failed file opening
	if (!chassisRightSpeeds || !chassisLeftSpeeds || !IMUValues || !chassisRightPos || !chassisLeftPos){
		con.clear();
		con.print(2, 0, "file open failed");
		return;
	}

	double startTime = pros::millis();
	while(pros::millis() - startTime < durationInSeconds*1000) {
		//record chassis speeds
		int16_t rightPos = static_cast<int16_t>((chassisFR.get_position() + chassisMR.get_position() + chassisBR.get_position()) / 3);
		int16_t leftPos = static_cast<int16_t>((chassisFL.get_position() + chassisML.get_position() + chassisBL.get_position()) / 3);
		fwrite(&rightPos, sizeof(rightPos), 1, chassisRightPos);
        fwrite(&leftPos, sizeof(leftPos), 1, chassisLeftPos);

		int16_t rightSpeed = static_cast<int16_t>(recordChassisRight(refreshRate));
		int16_t leftSpeed = static_cast<int16_t>(recordChassisLeft(refreshRate));
		fwrite(&rightPos, sizeof(rightSpeed), 1, chassisRightSpeeds);
        fwrite(&leftPos, sizeof(leftSpeed), 1, chassisLeftSpeeds);
		// chassisRightSpeeds.write(reinterpret_cast<const char*>(&rightSpeed), sizeof(rightSpeed));
		// chassisLeftSpeeds.write(reinterpret_cast<const char*>(&leftSpeed), sizeof(leftSpeed));

		//record imu heading
		uint16_t heading = static_cast<int16_t>(imu.get_heading());
		fwrite(&heading, sizeof(heading), 1, IMUValues);
		//IMUValues.write(reinterpret_cast<const char*>(&heading), sizeof(heading));
		//write takes two arguments, a pointer to the location of the data (where in memory is it stored), and the size of the data to be written
		//reinterpret_cast forces the compiler to treat the data (heading) as the specified value, which is const char* which is a constant pointer that points to a single byte of memory (what char denotes)
		//&heading means the address in memory where the value of heading is stored
		//so basically the code is finding the single byte of space in memory where the value of heading is and converting it to a pointer to be used in write()
		//sizeof finds the size of the var heading and returns it in bytes, satisfying the second argument of write()

		pros::delay(refreshRate);
	}

	fclose(chassisRightSpeeds);
    fclose(chassisLeftSpeeds);
    fclose(IMUValues);
	fclose(chassisRightPos);
	fclose(chassisLeftPos);
}



//###########################//
//		 READING             //
//       THE FILES           //
//      INTO A VECTOR        //
//###########################//
std::vector<double> rightSpeeds;
std::vector<double> leftSpeeds;
std::vector<double> headings;
std::vector<double> rightPos;
std::vector<double> leftPos;
void readPosFromSD(double durationInSeconds = 15, double refreshRate = 20) {
    // Open files on the SD card using PROS filesystem
    FILE* chassisRightSpeeds = fopen("/usd/chassisRightSpeeds.bin", "rb");
    FILE* chassisLeftSpeeds = fopen("/usd/chassisLeftSpeeds.bin", "rb");
    FILE* IMUValues = fopen("/usd/IMUValues.bin", "rb");
	FILE* chassisRightPos = fopen("/usd/chassisRightPos.bin", "rb");
	FILE* chassisLeftPos = fopen("/usd/chassisLeftPos.bin", "rb");

    if (!chassisRightSpeeds || !chassisLeftSpeeds || !IMUValues || !chassisRightPos || !chassisLeftPos) {
        pros::lcd::set_text(1, "file open failed");
        return;
    }

    double time = 0; // Used to track time progression
    int16_t rightSpeed;
    int16_t leftSpeed;
    uint16_t heading;
	uint32_t rightPosition;
	uint32_t leftPosition;

    while (time < (durationInSeconds * 1000)) {
        // Read binary data from files
        size_t rsRead = fread(&rightSpeed, sizeof(rightSpeed), 1, chassisRightSpeeds);
        size_t lsRead = fread(&leftSpeed, sizeof(leftSpeed), 1, chassisLeftSpeeds);
        size_t hRead = fread(&heading, sizeof(heading), 1, IMUValues);
		size_t rpRead = fread(&rightPos, sizeof(rightPos), 1, chassisRightPos);
		size_t lpRead = fread(&leftPos, sizeof(leftPos), 1, chassisLeftPos);
		//size t is a variable type that is basically size of something in bytes

        // Break if we reach the end of any file
        if (rsRead < 1 || lsRead < 1 || hRead < 1 || rpRead < 1 || lpRead < 1) {
            break;
			//fread returns the number of bytes successfully read so if its less than 1 that means u either reached the end of the file or something blew up
        }

        // Convert the read values to double and store in vectors
        rightSpeeds.push_back(static_cast<double>(rightSpeed));
        leftSpeeds.push_back(static_cast<double>(leftSpeed));
        headings.push_back(static_cast<double>(heading));
		rightPos.push_back(static_cast<double>(rightPosition));
		leftPos.push_back(static_cast<double>(leftPosition));
		//static cast to convert the int16_t to double

        time += refreshRate;
    }

    // Close files
    fclose(chassisRightSpeeds);
    fclose(chassisLeftSpeeds);
    fclose(IMUValues);
	fclose(chassisRightPos);
	fclose(chassisLeftPos);
}


//###########################//
//       READING THE		 //
//       VECTORS AND		 //
//       MOVING THE ROBOT	 //
//###########################//
double wantedRightSpeed;
double wantedLeftSpeed;
double wantedHeading;
double wantedRightPosition;
double wantedLeftPosition;
void playbackFromVector(double durationInSeconds=15, double refreshRate=20){
	double lastLoop = pros::millis();
	int index = 0;
	while (true){
		//if its been 20 miliseconds since the last loop update the wanted values
		if (pros::millis() - lastLoop >= refreshRate){
			//vectors are in ticks per refresh rate
			wantedRightSpeed = rightSpeeds[index];
			wantedLeftSpeed = leftSpeeds[index];
			wantedHeading = headings[index];
			wantedRightPosition = rightPos[index];
			wantedLeftPosition = leftPos[index];

			index++;
			lastLoop = pros::millis();
		}
		//run a pid to get the motor speeds to the desired ticks per RR - SPEED PID
		double rightSpeedPID = calculatePIDRight(wantedRightSpeed, recordChassisRight());
		double leftSpeedPID = calculatePIDLeft(wantedLeftSpeed, recordChassisLeft());

		//run a pid to get to the desired position - POSITION PID
		double rightPositionPID = calculatePIDRight(wantedRightPosition, ((chassisFR.get_position() + chassisMR.get_position() + chassisBR.get_position()) / 3));
		double leftPositionPID = calculatePIDLeft(wantedLeftPosition, ((chassisFL.get_position() + chassisML.get_position() + chassisBL.get_position()) / 3));
		
		//heading correction using the desired imu value
		double headingCorrection = calculatePIDHC(wantedHeading, imu.get_heading());


		//clamp the distance pid output to -30 to 30, so corrects for distance do not overrride the speed pid
		//essentially, now the the speed pid will run and it will go 30 volts faster or slower (depending on distance pid) to correct for distance
		double clampedRightDistancePID = std::clamp(rightPositionPID, -30.0, 30.0);
		double clampedLeftDistancePID = std::clamp(leftPositionPID, -30.0, 30.0);

		//add the distance pid output to the speed pid output
		double rightSpeed = rightSpeedPID + clampedRightDistancePID;
		double leftSpeed = leftSpeedPID + clampedLeftDistancePID;

		//run the motors
		chassisFR.move(rightSpeed - headingCorrection);
		chassisMR.move(rightSpeed - headingCorrection);
		chassisBR.move(rightSpeed - headingCorrection);

		chassisFL.move(leftSpeed + headingCorrection);
		chassisML.move(leftSpeed + headingCorrection);
		chassisBL.move(leftSpeed + headingCorrection);
	}
}



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
	con.clear();
	while (true){
		//select something to do
		con.print(0,0, "A->Record");
		con.print(1,0, "B->Playback");

		if (con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			con.rumble("."); //buzz when starting recording
			//start writing, you're gonna need a thread for this
		}
	}
}
