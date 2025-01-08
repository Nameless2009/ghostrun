#ifndef __RECORDER__
#define __RECORDER__

#include "main.h"
#include "global.h"

//###########################//
//         DEVICE            //
//         ACTIVITY          //
//         RECORDER          //
//###########################//
extern double prevChassisRight = 0;
extern double recordChassisRight(double refreshRate=20);

extern double prevChassisLeft = 0;
extern double recordChassisLeft(double refreshRate=20);

#endif