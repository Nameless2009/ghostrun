//has all the backround work, storing obecys, basic framework

#ifndef _GLOBAL_
#define _GLOBAL_

#include "main.h"

using namespace pros;

namespace global {

    #define P_L1 8  //Reversed
    #define P_L2 9  //Reversed
    #define P_L3 10 //Not reversed
    #define P_R1 2  //Reversed
    #define P_R2 3  //Reversed
    #define P_R3 4  //Not reversed
    #define P_Intake 6
    #define P_LadyBrown 7
    #define A_Clamp 'A'
    #define B_Inertial 'B'
    //ports for all motors, for sensors and ports
    // "A" replace 

    //extern makes it usable for other files
    extern pros::Motor L1;
    extern pros::Motor L2;
    extern pros::Motor L3;
    extern pros::Motor R1;
    extern pros::Motor R2;
    extern pros::Motor R3;
    extern pros::Motor Intake;
    extern pros::Motor LadyBrown;
    extern pros::ADIDigitalOut Clamp;
    extern pros::IMU Inertial;
    
    extern pros::Controller DriversInput;

    extern pros::Motor_Group LC;
    extern pros::Motor_Group RC;

}

//Lady Brown 7, Left -8 -9 10 Right -2 -3 4

#endif
