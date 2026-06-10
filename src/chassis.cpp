#include "chassis.h"
#include "parameters.h"
#include "sensors.h"
#include "pid.h"
#include "math-utils.h"
#include "autonomous.h"
#include <algorithm>
#include <thread>

using namespace vex;

namespace chassis {

static PID fwdPID(ChassisPID::kFwdConfig);
static PID rotPID(ChassisPID::kRotConfig);
static PID leftPID(ChassisPID::kCurveConfig);
static PID rightPID(ChassisPID::kCurveConfig);


void spinFL(float power)   { Motor_FL.spin(fwd, power, pct); }
void spinFR(float power)   { Motor_FR.spin(fwd, power, pct); }
void spinBL(float power)   { Motor_BL.spin(fwd, power, pct); }
void spinBR(float power)   { Motor_BR.spin(fwd, power, pct); }
void spinAuxL(float power) { Motor_AuxL.spin(fwd, power, pct); }
void spinAuxR(float power) { Motor_AuxR.spin(fwd, power, pct); }



void moveLeft(float power) {
    spinAuxL(power); spinFL(power); spinBL(power);
}

void moveRight(float power) {
    spinFR(power); spinBR(power); spinAuxR(power);
}

void moveForward(float power) {
    spinAuxL(power); spinFL(power); spinFR(power);
    spinBL(power);  spinBR(power); spinAuxR(power);
}

void moveRotate(float power) {
    spinAuxL(power);  spinFL(power);  spinFR(-power);
    spinBL(power);    spinBR(-power); spinAuxR(-power);
}


void brakeAll() {
    Motor_AuxL.stop(brake); Motor_FL.stop(brake); Motor_FR.stop(brake);
    Motor_BL.stop(brake);  Motor_BR.stop(brake); Motor_AuxR.stop(brake);
}

void coastAll() {
    Motor_AuxL.stop(coast); Motor_FL.stop(coast); Motor_FR.stop(coast);
    Motor_BL.stop(coast);  Motor_BR.stop(coast); Motor_AuxR.stop(coast);
}

void lockBase() {
    Motor_AuxL.stop(hold); Motor_FL.stop(hold); Motor_FR.stop(hold);
    Motor_BL.stop(hold);  Motor_BR.stop(hold); Motor_AuxR.stop(hold);
}

void unlockBase() {
    Motor_AuxL.stop(coast); Motor_FL.stop(coast); Motor_FR.stop(coast);
    Motor_BL.stop(coast);  Motor_BR.stop(coast); Motor_AuxR.stop(coast);
}

void lockLeft() {
    Motor_AuxL.stop(hold); Motor_FL.stop(hold); Motor_BL.stop(hold);
}

void unlockLeft() {
    Motor_AuxL.stop(coast); Motor_FL.stop(coast); Motor_BL.stop(coast);
}

void lockRight() {
    Motor_FR.stop(hold); Motor_BR.stop(hold); Motor_AuxR.stop(hold);
}

void unlockRight() {
    Motor_FR.stop(coast); Motor_BR.stop(coast); Motor_AuxR.stop(coast);
}
void arcadeDrive(int forward, int turn) {
    int absFwd  = std::abs(forward);
    int absTurn = std::abs(turn);
    int sum     = absFwd + absTurn;
    if (sum > kDriveMaxCombinedPower) {
        forward = forward * kDriveMaxCombinedPower / sum;
        turn    = turn    * kDriveMaxCombinedPower / sum;
    }

    int fl   = forward + turn;
    int fr   = forward - turn;
    int bl   = forward + turn;
    int br   = forward - turn;
    int auxL = forward + turn;
    int auxR = forward - turn;

    spinAuxL(static_cast<float>(auxL));
    spinFL(static_cast<float>(fl));
    spinFR(static_cast<float>(fr));
    spinBL(static_cast<float>(bl));
    spinBR(static_cast<float>(br));
    spinAuxR(static_cast<float>(auxR));
}



float getForwardPos() {
    float avg = (Motor_FL.position(deg) + Motor_FR.position(deg) +
                 Motor_BL.position(deg) + Motor_BR.position(deg)) / 4.0f;
    return avg / Encoder::kDegPerRev * Physics::kWheelCircumference * Physics::kMmPerInch;
}

float getLeftPos() {
    return (Motor_AuxL.position(deg) + Motor_FL.position(deg) +
            Motor_BL.position(deg)) / 3.0f;
}

float getRightPos() {
    return (Motor_FR.position(deg) + Motor_BR.position(deg) +
            Motor_AuxR.position(deg)) / 3.0f;
}

void resetForwardPos() {
    Motor_AuxL.resetPosition(); Motor_FL.resetPosition();
    Motor_FR.resetPosition();   Motor_BL.resetPosition();
    Motor_BR.resetPosition();   Motor_AuxR.resetPosition();
}

float getMaxDriveTemp() {
    float t1 = Motor_AuxL.temperature(celsius);
    float t2 = Motor_FL.temperature(celsius);
    float t3 = Motor_FR.temperature(celsius);
    float t4 = Motor_BL.temperature(celsius);
    float t5 = Motor_BR.temperature(celsius);
    float t6 = Motor_AuxR.temperature(celsius);
    float maxT = t1;
    if (t2 > maxT) maxT = t2;
    if (t3 > maxT) maxT = t3;
    if (t4 > maxT) maxT = t4;
    if (t5 > maxT) maxT = t5;
    if (t6 > maxT) maxT = t6;
    return maxT;
}


void softStartTimerForward(float pwrInit, float pwrFinal, int durationMs) {
    MyTimer t;
    while (t.getTime() < durationMs) {
        float progress = static_cast<float>(t.getTime()) / durationMs;
        float power = pwrInit + (pwrFinal - pwrInit) * progress;
        moveForward(power);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}

void timerForward(float power, int durationMs,
                  float targetHeading, float distanceLimit) {
    MyTimer t;
    float start = getForwardPos();
    while (t.getTime() < durationMs) {
        if (distanceLimit > 0 &&
            fabs(getForwardPos() - start) >= distanceLimit) break;
        driveWithHeading(power, targetHeading);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}

void timerRotate(float power, int durationMs) {
    MyTimer t;
    while (t.getTime() < durationMs) {
        moveRotate(power);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}


void posForwardAbs(float power, float target, float targetHeading) {
    resetForwardPos();
    int dir = (target > 0) ? 1 : -1;
    float remaining = fabs(target);
    while (remaining > Motion::kPosRemainingThreshold) {
        remaining = fabs(target) - fabs(getForwardPos());
        if (remaining <= 0) break;
        float adj = power * math::clamp(remaining / Motion::kPosDecelDistance,
                                        Motion::kPosMinPowerRatio,
                                        Motion::kPosMaxPowerRatio);
        driveWithHeading(dir * adj, targetHeading);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}

void angleRotateAbs(float power, float target) {
    float error = math::normalizeAngle(target - getHeading());
    int dir = (error > 0) ? 1 : -1;
    while (fabs(error) > Motion::kAngleRemainingThreshold) {
        error = math::normalizeAngle(target - getHeading());
        float adj = power * math::clamp(fabs(error) / Motion::kAngleDecelAngle,
                                        Motion::kAngleMinPowerRatio,
                                        Motion::kAngleMaxPowerRatio);
        moveRotate(dir * adj);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}


void pidForwardAbs(float target, float errorTolerance) {
    resetForwardPos();
    fwdPID.setTarget(target);
    fwdPID.setErrorTolerance(errorTolerance);
    fwdPID.setFirstTime();
    while (!fwdPID.targetArrived()) {
        if (!auto_is_active()) { brakeAll(); return; }
        fwdPID.update(getForwardPos());
        float out = math::clampSym(fwdPID.getOutput(), ChassisPID::kFwdOutputLimit);
        moveForward(out);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}

void pidForwardAbs(float target, float kp, float ki, float kd, float errorTolerance) {
    fwdPID.setCoefficient(kp, ki, kd);
    pidForwardAbs(target, errorTolerance);
}

void pidRotateAbs(float target, float errorTolerance) {
    rotPID.setTarget(target);
    rotPID.setErrorTolerance(errorTolerance);
    rotPID.setFirstTime();
    while (!rotPID.targetArrived()) {
        if (!auto_is_active()) { brakeAll(); return; }
        float input = getHeading();
        float error = math::normalizeAngle(target - input);
        float fakeInput = target - error;
        rotPID.update(fakeInput);
        float out = math::clampSym(rotPID.getOutput(), ChassisPID::kRotOutputLimit);
        moveRotate(out);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}

void pidRotateAbs(float target, float kp, float ki, float kd, float errorTolerance) {
    rotPID.setCoefficient(kp, ki, kd);
    pidRotateAbs(target, errorTolerance);
}

void posCurve(float leftPwr, float rightPwr, float target, bool mirror) {
    resetForwardPos();
    float remaining = fabs(target);
    while (remaining > Motion::kPosRemainingThreshold) {
        remaining = fabs(target) - fabs(getForwardPos());
        if (remaining <= 0) break;
        float scale = math::clamp(remaining / Motion::kPosDecelDistance,
                                  Motion::kPosMinPowerRatio,
                                  Motion::kPosMaxPowerRatio);
        moveLeft(leftPwr * scale);
        moveRight(rightPwr * scale);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}

void PIDPosCurveAbs(float leftTarget, float rightTarget, float tolerance) {
    resetForwardPos();
    leftPID.setTarget(leftTarget);
    leftPID.setErrorTolerance(tolerance);
    leftPID.setFirstTime();
    rightPID.setTarget(rightTarget);
    rightPID.setErrorTolerance(tolerance);
    rightPID.setFirstTime();
    while (!leftPID.targetArrived() || !rightPID.targetArrived()) {
        if (!auto_is_active()) { brakeAll(); return; }
        leftPID.update(getLeftPos());
        rightPID.update(getRightPos());
        float lOut = math::clampSym(leftPID.getOutput(), ChassisPID::kCurveOutputLimit);
        float rOut = math::clampSym(rightPID.getOutput(), ChassisPID::kCurveOutputLimit);
        moveLeft(lOut);
        moveRight(rOut);
        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}

void PIDPosCurveAbs(float leftTarget, float rightTarget, float kp, float ki, float kd, float tolerance) {
    leftPID.setCoefficient(kp, ki, kd);
    rightPID.setCoefficient(kp, ki, kd);
    PIDPosCurveAbs(leftTarget, rightTarget, tolerance);
}


void driveWithHeading(float power, float targetHeading) {
    float error = math::normalizeAngle(targetHeading - getHeading());
    float correction = error * Motion::kHeadingCorrectionGain;
    correction = math::clampSym(correction, Motion::kHeadingCorrectionMax);
    int fwd = static_cast<int>(power);
    int turn = static_cast<int>(correction);
    arcadeDrive(fwd, turn);
}

}
