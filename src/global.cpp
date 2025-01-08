#include "main.h"
#include "global.h"

namespace glb{
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
}