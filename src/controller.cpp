#include "controller.h"
#include "config.h"
#include <thread>

using namespace vex;

double ctrlTimeMs = 0;

int joyA1 = 0, joyA2 = 0, joyA3 = 0, joyA4 = 0;

bool btnL1 = false, btnL2 = false, btnR1 = false, btnR2 = false;
bool btnX = false, btnY = false, btnA = false, btnB = false;
bool btnLeft = false, btnRight = false, btnUp = false, btnDown = false;

bool lastL1 = false, lastL2 = false, lastR1 = false, lastR2 = false;
bool lastX = false, lastY = false, lastA = false, lastB = false;
bool lastLeft = false, lastRight = false, lastUp = false, lastDown = false;
void defineController() {
    lastL1 = btnL1;   lastL2 = btnL2;
    lastR1 = btnR1;   lastR2 = btnR2;
    lastX  = btnX;    lastY  = btnY;
    lastA  = btnA;    lastB  = btnB;
    lastLeft  = btnLeft;  lastRight = btnRight;
    lastUp    = btnUp;    lastDown  = btnDown;

  
    ctrlTimeMs = Brain.Timer.value() * 1000.0;


    joyA1 = Controller.Axis1.position();  
    joyA3 = Controller.Axis3.position();  
    btnL1    = Controller.ButtonL1.pressing();
    btnL2    = Controller.ButtonL2.pressing();
    btnR1    = Controller.ButtonR1.pressing();
    btnR2    = Controller.ButtonR2.pressing();
    btnX     = Controller.ButtonX.pressing();
    btnY     = Controller.ButtonY.pressing();
    btnA     = Controller.ButtonA.pressing();
    btnB     = Controller.ButtonB.pressing();
    btnLeft  = Controller.ButtonLeft.pressing();
    btnRight = Controller.ButtonRight.pressing();
    btnUp    = Controller.ButtonUp.pressing();
    btnDown  = Controller.ButtonDown.pressing();

    this_thread::sleep_for(Timing::kControllerPollInterval);
}
