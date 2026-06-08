#include "odometry.h"
#include "config.h"
#include "sensors.h"
#include "parameters.h"
#include "math-utils.h"
#include <cmath>

using namespace vex;

namespace odometry {

static float gX = 0.0f, gY = 0.0f;       
static float lastX = 0.0f, lastY = 0.0f;  
static bool  initialized = false;

void init(float x, float y) {
    gX = x;  gY = y;
    lastX = Odometer[0].position(deg);
    lastY = Odometer[1].position(deg);
    initialized = true;
}

float rawX() { return Odometer[0].position(deg); }
float rawY() { return Odometer[1].position(deg); }

float globalX() { return gX; }
float globalY() { return gY; }

float heading() { return getHeading(); }

void setPosition(float x, float y) { gX = x;  gY = y; }

bool calibrated() { return initialized; }


static float degToInch(float deltaDeg) {
    return (deltaDeg / kOdometryTicksPerRev) * kOdometryWheelCirc;
}

void updateLoop() {
    while (!initialized) {
        this_thread::sleep_for(Timing::kOdometryLoopInterval);
    }

    while (true) {
        float curX = Odometer[0].position(deg);
        float curY = Odometer[1].position(deg);

        float dLocalX = degToInch(curX - lastX);
        float dLocalY = degToInch(curY - lastY);

        lastX = curX;
        lastY = curY;

        float hdg = math::deg2rad(getHeading());

        float cosH = cosf(hdg);
        float sinH = sinf(hdg);
        float dGlobalX = dLocalX * cosH - dLocalY * sinH;
        float dGlobalY = dLocalX * sinH + dLocalY * cosH;

        gX += dGlobalX;
        gY += dGlobalY;

        this_thread::sleep_for(Timing::kOdometryLoopInterval);  // ~100Hz
    }
}

}  
