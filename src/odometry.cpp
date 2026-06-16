#include "odometry.h"
#include "config.h"
#include "sensors.h"
#include "parameters.h"
#include "math-utils.h"
#include <cmath>

using namespace vex;

namespace odometry {

static float gX = 0.0f, gY = 0.0f;

static float rGX = 0.0f, rGY = 0.0f;

static float lastX = 0.0f, lastY = 0.0f;
static float lastHdg = 0.0f;
static bool  initialized = false;
static bool  stuck = false;
static int   stuckCounter = 0;

static float emaRgx = 0.0f, emaRgy = 0.0f;


static int   stuckCooldown = 0;

static void resetTracking() {
    lastX = Odometer[0].position(deg);
    lastY = Odometer[1].position(deg);
    lastHdg = getHeading();
    stuck = false;
    stuckCounter = 0;
    stuckCooldown = 0;
}

void init(float x, float y) {
    gX = x;  gY = y;
    rGX = gX;  rGY = gY;
    emaRgx = rGX;  emaRgy = rGY;
    resetTracking();
    initialized = true;
}

float rawX() { return Odometer[0].position(deg); }
float rawY() { return Odometer[1].position(deg); }

float globalX() { return gX; }
float globalY() { return gY; }
float globalXMm() { return gX * Physics::kMmPerInch; }
float globalYMm() { return gY * Physics::kMmPerInch; }

float robotGlobalX() { return rGX; }
float robotGlobalY() { return rGY; }
float robotGlobalXMm() { return rGX * Physics::kMmPerInch; }
float robotGlobalYMm() { return rGY * Physics::kMmPerInch; }

float heading() { return getHeading(); }

void setPosition(float x, float y) {
    gX = x;  gY = y;
    rGX = gX;  rGY = gY;
    emaRgx = rGX;  emaRgy = rGY;
}

void setPositionMm(float xMm, float yMm) {
    gX = xMm / Physics::kMmPerInch;
    gY = yMm / Physics::kMmPerInch;
    rGX = gX;  rGY = gY;
    emaRgx = rGX;  emaRgy = rGY;
}

bool calibrated() { return initialized; }
bool isStuck()    { return stuck; }

void calibrate() { resetTracking(); }

static float degToInch(float deltaDeg) {
    return (deltaDeg / kOdometryTicksPerRev) * kOdometryWheelCirc;
}


static float arcDisplacement(float dWheel, float dTheta, float offset) {
    if (fabsf(dTheta) < Odometry::kArcEpsilonRad) {
        return dWheel + offset * dTheta;
    }
    float radius = dWheel / dTheta + offset;
    return 2.0f * sinf(dTheta * 0.5f) * radius;
}

void updateLoop() {
    while (!initialized) {
        this_thread::sleep_for(Timing::kOdometryLoopInterval);
    }

    while (true) {

        float curX = Odometer[0].position(deg);
        float curY = Odometer[1].position(deg);
        float curHdg = getHeading();

    
        float dWheelX = degToInch(curX - lastX);   
        float dWheelY = degToInch(curY - lastY);   

        float dHdgDeg = curHdg - lastHdg;
        float dHdgRad = math::deg2rad(dHdgDeg);

  
        float dLocalX = arcDisplacement(dWheelX, dHdgRad,
                                        Odometry::kFwdWheelOffsetY);


        float dLocalY = arcDisplacement(dWheelY, dHdgRad,
                                        -Odometry::kStrWheelOffsetX);


        float hdgMid = math::deg2rad(lastHdg + dHdgDeg * 0.5f);
        float cosH = cosf(hdgMid);
        float sinH = sinf(hdgMid);

        float dGlobalX = dLocalX * cosH - dLocalY * sinH;
        float dGlobalY = dLocalX * sinH + dLocalY * cosH;

        gX += dGlobalX;
        gY += dGlobalY;

        float rawRgx = rGX + dGlobalX;
        float rawRgy = rGY + dGlobalY;

        bool isStationary = (fabsf(dHdgDeg) < 0.05f &&
                             fabsf(dWheelX) < Odometry::kStuckThreshold &&
                             fabsf(dWheelY) < Odometry::kStuckThreshold);

        if (isStationary) {

            float corr = (1.0f - Odometry::kDriftCorrGain);
            float dampedX = rGX + dGlobalX * corr;
            float dampedY = rGY + dGlobalY * corr;

            rGX = Odometry::kEmaAlpha * dampedX
                + (1.0f - Odometry::kEmaAlpha) * rGX;
            rGY = Odometry::kEmaAlpha * dampedY
                + (1.0f - Odometry::kEmaAlpha) * rGY;

            emaRgx = rGX;
            emaRgy = rGY;
        } else {
            emaRgx = Odometry::kEmaAlpha * rawRgx
                   + (1.0f - Odometry::kEmaAlpha) * emaRgx;
            emaRgy = Odometry::kEmaAlpha * rawRgy
                   + (1.0f - Odometry::kEmaAlpha) * emaRgy;

            rGX = emaRgx;
            rGY = emaRgy;
        }

        bool currentlyStuck = (fabsf(dWheelX) < Odometry::kStuckThreshold &&
                               fabsf(dWheelY) < Odometry::kStuckThreshold &&
                               fabsf(dHdgDeg) < 0.05f);

        if (stuckCooldown > 0) {
            stuckCooldown--;
        }

        if (!stuck && currentlyStuck && stuckCooldown == 0) {
            stuckCounter++;
            if (stuckCounter >= Odometry::kStuckFrames) {
                stuck = true;
                stuckCounter = 0;
            }
        } else if (stuck && !currentlyStuck) {
            if (fabsf(dWheelX) > Odometry::kStuckRecoveryThreshold ||
                fabsf(dWheelY) > Odometry::kStuckRecoveryThreshold ||
                fabsf(dHdgDeg) > 0.1f) {
                stuck = false;
                stuckCounter = 0;
                stuckCooldown = Odometry::kStuckCooldownFrames;
            }
        } else if (!currentlyStuck) {
            stuckCounter = 0;
        }

        lastX = curX;
        lastY = curY;
        lastHdg = curHdg;

        this_thread::sleep_for(Timing::kOdometryLoopInterval);
    }
}

}
