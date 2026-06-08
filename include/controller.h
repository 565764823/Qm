#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "config.h"

using namespace vex;



extern double ctrlTimeMs;  
extern int joyA1, joyA2, joyA3, joyA4;
extern bool btnL1, btnL2, btnR1, btnR2;
extern bool btnX,  btnY,  btnA,  btnB;
extern bool btnLeft, btnRight, btnUp, btnDown;

extern bool lastL1, lastL2, lastR1, lastR2;
extern bool lastX,  lastY,  lastA,  lastB;
extern bool lastLeft, lastRight, lastUp, lastDown;
void defineController();

#endif  // CONTROLLER_H_
