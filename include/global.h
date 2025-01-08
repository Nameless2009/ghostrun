#ifndef __GLOBAL__
#define __GLOBAL__

#include "main.h"
#include <vector>
#include <string>
#include <cstdio>
#include "unistd.h"

namespace glb{
    #define P_FR 1
    #define P_FL 2
    #define P_MR 3
    #define P_ML 4
    #define P_BR 5
    #define P_BL 6

    #define P_IMU 7

    //###########################//
    //         DEVICE            //
    //         CONFIG            //
    //###########################//
    extern pros::Motor chassisFR;
    extern pros::Motor chassisFL;
    extern pros::Motor chassisMR;
    extern pros::Motor chassisML;
    extern pros::Motor chassisBR;
    extern pros::Motor chassisBL;

    extern pros::IMU imu;

    extern pros::Controller con;
}


#endif