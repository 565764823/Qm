#include "autonomous.h"
#include "chassis.h"
#include "config.h"
#include "math-utils.h"
#include "odometry.h"
#include "sensors.h"

PIDConfig autoFwd   = AutoPID::kFwd;
PIDConfig autoRot   = AutoPID::kRot;
PIDConfig autoCurve = AutoPID::kCurve;

int auton_strategy = 0;

bool g_auto_active = false;

static bool s_auto_lastB = false;

void auto_notify_start() {
    s_auto_lastB = Controller.ButtonB.pressing();
}

bool auto_is_active() {
    if (!g_auto_active) return false;

    bool currB = Controller.ButtonB.pressing();
    if (currB && !s_auto_lastB) {
        g_auto_active = false;
    }
    s_auto_lastB = currB;
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
        if (fabs(err) > Motion::kHeadingCorrDeadband)
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


static void route_right() {
    odometry::setPositionMm(0, 0);

    chassis::gpsMove(600, -1200, 1, 50, "PID");

    chassis::gpsMove(1800, -600, 1, 60, "PID");

    chassis::gpsMove(200, 0, -1, 50, "PID");
}

static void route_left() {
    odometry::setPositionMm(0, 0);

    chassis::gpsMove(600, 1200, 1, 50, "PID");

    chassis::gpsMove(1800, 600, 1, 60, "PID");

    chassis::pidForwardRel(400, 5.0f);

    chassis::gpsMove(200, 0, -1, 50, "PID");
}

static void route_skill() {
    odometry::setPositionMm(0, 0);

    chassis::gpsMove(600, -800, 1, 50, "PID");

    chassis::gpsMove(1200, -1200, 1, 60, "continue");

    chassis::gpsAim(2400, 0, 1, 0);

    chassis::gpsMove(1800, 0, 1, 60, "PID");

    chassis::gpsMove(1200, 800, 1, 50, "PID");

    chassis::gpsMove(2400, 400, 1, 60, "PID");

    chassis::gpsMove(600, 0, -1, 50, "heading");
}


void auto_run() {
    chassis::resetForwardPos();

    g_auto_active = true;
    auto_notify_start();  

    switch (auton_strategy) {
    case 0: route_demo();     break;
    case 1: route_collect();  break;
    case 2: route_slalom();   break;
    case 3: route_right();    break;
    case 4: route_left();     break;
    case 5: route_skill();    break;
    }

    g_auto_active = false;
    chassis::brakeAll();
}

int auto_cycle() {
    auton_strategy = (auton_strategy + 1) % Auto::kRouteCount;
    return auton_strategy;
}

const char* auto_name() {
    switch (auton_strategy) {
    case 0: return "demo";
    case 1: return "collect";
    case 2: return "slalom";
    case 3: return "right (GPS)";
    case 4: return "left  (GPS)";
    case 5: return "skill (GPS)";
    default: return "?";
    }
}
