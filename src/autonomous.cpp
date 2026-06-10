#include "autonomous.h"
#include "chassis.h"
#include "config.h"
#include "math-utils.h"
#include "sensors.h"

PIDConfig autoFwd   = AutoPID::kFwd;
PIDConfig autoRot   = AutoPID::kRot;
PIDConfig autoCurve = AutoPID::kCurve;

#define AUTON_ROUTES 3

int auton_strategy = 0;

bool g_auto_active = false;

bool auto_is_active() {
    if (!g_auto_active) return false;

    static bool lastB = false;
    bool currB = Controller.ButtonB.pressing();
    if (currB && !lastB) {
        g_auto_active = false;
    }
    lastB = currB;
    return g_auto_active;
}

void auto_resetPID() {
    autoFwd   = AutoPID::kFwd;
    autoRot   = AutoPID::kRot;
    autoCurve = AutoPID::kCurve;
}

static inline void go(float mm, float heading = 0) {
    chassis::pidForwardAbs(mm, autoFwd.kp, autoFwd.ki, autoFwd.kd);
    if (heading != 0) {
        float err = math::normalizeAngle(heading - getHeading());
        if (fabs(err) > 1.0f)
            chassis::pidRotateAbs(heading, autoRot.kp, autoRot.ki, autoRot.kd);
    }
}

static inline void turn(float deg) {
    chassis::pidRotateAbs(deg, autoRot.kp, autoRot.ki, autoRot.kd);
}

static inline void curve(float l, float r) {
    chassis::PIDPosCurveAbs(l, r, autoCurve.kp, autoCurve.ki, autoCurve.kd);
}
static void route_demo() {
    autoFwd = AutoPID::kFwdFast;
    go(800);
    turn(90);
    autoFwd = AutoPID::kFwdSlow;
    go(200);
    auto_resetPID();
    go(300);
}

static void route_collect() {
    go(600);   turn(45);     
    go(400);   turn(-90);    
    go(-300);  turn(45);     
    go(500);                 
}

static void route_slalom() {
    go(300);                     
    curve(600, 300);              
    go(200);                       
    curve(300, 600);               
    go(300);
}


void auto_run() {
    chassis::resetForwardPos();

    g_auto_active = true;

    switch (auton_strategy) {
    case 0: route_demo();     break;
    case 1: route_collect();  break;
    case 2: route_slalom();   break;
    }

    g_auto_active = false;
    chassis::brakeAll();
}

int auto_cycle() {
    auton_strategy = (auton_strategy + 1) % AUTON_ROUTES;
    return auton_strategy;
}
