#ifndef __DATA__
#define __DATA__

#include "main.h"
#include "global.h"

//###########################//
//		 WRITING             //
//       TO FILES            //
//###########################//
extern void writeToSD(double durationInSeconds=15, double refreshRate=20);



//###########################//
//		 READING             //
//       THE FILES           //
//      INTO A VECTOR        //
//###########################//
extern std::vector<double> rightSpeeds;
extern std::vector<double> leftSpeeds;
extern std::vector<double> headings;
extern std::vector<double> rightPos;
extern std::vector<double> leftPos;
extern void readPosFromSD(double durationInSeconds = 15, double refreshRate = 20);



//###########################//
//       READING THE		 //
//       VECTORS AND		 //
//       MOVING THE ROBOT	 //
//###########################//
extern double wantedRightSpeed;
extern double wantedLeftSpeed;
extern double wantedHeading;
extern double wantedRightPosition;
extern double wantedLeftPosition;
extern void playbackFromVector(double durationInSeconds=15, double refreshRate=20);


#endif