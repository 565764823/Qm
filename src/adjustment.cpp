#include "adjustment.h"
#include "autonomous.h"
#include "chassis.h"
#include "config.h"
#include "math-utils.h"
#include "parameters.h"
#include "pid.h"
#include "sensors.h"
#include "timer.h"
#include <cstdio>
#include <thread>

using namespace vex;


static void drawHeader(const char* title) {
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono15);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("=== %s ===", title);
}

static void drawParams(float kp, float ki, float kd) {
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Kp: %+6.2f  [L1/L2]", kp);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Ki: %+6.2f  [R1/R2]", ki);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Kd: %+6.2f  [</>]", kd);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("微调: 按住L2+R2组合键");
}

static void drawResult(float elapsedSec, float error, const char* unit) {
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("耗时: %5.2fs  误差: %+6.1f%s",
                       elapsedSec, error, unit);
}
static int tuningStep(float &kp, float &ki, float &kd,
                      bool &lL1, bool &lL2, bool &lR1, bool &lR2,
                      bool &lL,  bool &lR,  bool &lA,  bool &lB) {
    bool L1 = Controller.ButtonL1.pressing();
    bool L2 = Controller.ButtonL2.pressing();
    bool R1 = Controller.ButtonR1.pressing();
    bool R2 = Controller.ButtonR2.pressing();
    bool L  = Controller.ButtonLeft.pressing();
    bool R  = Controller.ButtonRight.pressing();
    bool A  = Controller.ButtonA.pressing();
    bool B  = Controller.ButtonB.pressing();

    bool fine = (L2 && R2);

    if (L1 && !lL1) kp += fine ? -Tuning::kKpStepFine : -Tuning::kKpStep;
    if (L2 && !lL2 && !fine) kp += Tuning::kKpStep;
    if (R1 && !lR1) ki += fine ? -Tuning::kKiStepFine : -Tuning::kKiStep;
    if (R2 && !lR2 && !fine) ki += Tuning::kKiStep;
    if (L  && !lL)  kd += fine ? -Tuning::kKdStepFine : -Tuning::kKdStep;
    if (R  && !lR)  kd += fine ?  Tuning::kKdStepFine :  Tuning::kKdStep;

    if (kp < 0.0f) kp = 0.0f;
    if (ki < 0.0f) ki = 0.0f;
    if (kd < 0.0f) kd = 0.0f;

    drawParams(kp, ki, kd);

    bool runTest = (A && !lA);
    bool exitFn  = (B && !lB);

    lL1 = L1; lL2 = L2; lR1 = R1; lR2 = R2;
    lL  = L;  lR  = R;  lA  = A;  lB  = B;

    this_thread::sleep_for(Timing::kUserLoopInterval);

    if (exitFn)  return -1;
    if (runTest) return  1;
    return 0;
}

void tuningForward() {
    float kp = AutoPID::kFwdKp, ki = AutoPID::kFwdKi, kd = AutoPID::kFwdKd;
    bool lL1 = false, lL2 = false, lR1 = false, lR2 = false;
    bool lL = false, lR = false, lA = false, lB = false;

    drawHeader("TUNE FORWARD PID");

    while (true) {
        Brain.Screen.setCursor(11, 1);
        Brain.Screen.print("A: 测试 %.0fmm   B: 退出", Tuning::kTestForwardMm);

        int act = tuningStep(kp, ki, kd, lL1, lL2, lR1, lR2, lL, lR, lA, lB);
        if (act < 0) break;
        if (act == 0) continue;


        g_auto_active = true;
        auto_notify_start();

        chassis::resetForwardPos();
        MyTimer timer;
        chassis::pidForwardAbs(Tuning::kTestForwardMm,
                               kp, ki, kd, Tuning::kTestErrorTol);
        float elapsed = timer.getTimeSec();
        float error = fabs(Tuning::kTestForwardMm -
                           chassis::getForwardPos());
        drawResult(elapsed, error, "mm");

        g_auto_active = false;
        lB = Controller.ButtonB.pressing();
    }
    Brain.Screen.clearScreen();
}

void tuningRotate() {
    float kp = AutoPID::kRotKp, ki = AutoPID::kRotKi, kd = AutoPID::kRotKd;
    bool lL1 = false, lL2 = false, lR1 = false, lR2 = false;
    bool lL = false, lR = false, lA = false, lB = false;

    drawHeader("TUNE ROTATE PID");

    while (true) {
        Brain.Screen.setCursor(11, 1);
        Brain.Screen.print("A: 测试 %.0f%c   B: 退出",
                           Tuning::kTestRotateDeg, 0xB0);

        int act = tuningStep(kp, ki, kd, lL1, lL2, lR1, lR2, lL, lR, lA, lB);
        if (act < 0) break;
        if (act == 0) continue;

        g_auto_active = true;
        auto_notify_start();

        float startHdg = getHeading();
        MyTimer timer;
        chassis::pidRotateAbs(startHdg + Tuning::kTestRotateDeg,
                              kp, ki, kd, Tuning::kTestErrorTol);
        float elapsed = timer.getTimeSec();
        float error = fabs(Tuning::kTestRotateDeg -
                           math::normalizeAngle(getHeading() - startHdg));
        drawResult(elapsed, error, "deg");

        g_auto_active = false;
        lB = Controller.ButtonB.pressing();
    }
    Brain.Screen.clearScreen();
}
