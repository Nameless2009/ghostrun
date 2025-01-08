#include "main.h"
#include "data.h"
using namespace glb;

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
