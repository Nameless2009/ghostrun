//has all the backround work, storing obecys, basic framework

#ifndef _GLOBAL_
#define _GLOBAL_

#include "main.h"

using namespace pros;

namespace global {

    #define P_L1 1
    #define P_L2 2
    #define P_L3 3
    #define P_R1 4
    #define P_R2 5
    #define P_R3 6
    #define P_Intake 7
    #define P_LadyBrown 8
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
