#ifndef CHASSIS_H_
#define CHASSIS_H_
#include "config.h"
namespace chassis {
void spinFL(float power);
void spinFR(float power);
void spinBL(float power);
void spinBR(float power);
void spinAuxL(float power);
void spinAuxR(float power);
void moveLeft(float power);
void moveRight(float power);
void moveForward(float power);
void moveRotate(float power);
void spinFLVel(float velPct);
void spinFRVel(float velPct);
void spinBLVel(float velPct);
void spinBRVel(float velPct);
void spinAuxLVel(float velPct);
void spinAuxRVel(float velPct);
void moveLeftVel(float velPct);
void moveRightVel(float velPct);
void moveForwardVel(float velPct);
void moveRotateVel(float velPct);
void brakeAll();
void coastAll();
void lockBase();
void unlockBase();
void lockLeft();
void unlockLeft();
void lockRight();
void unlockRight();
void arcadeDrive(int forward, int turn);
float getForwardPos();
float getLeftPos();
float getRightPos();
float getForwardVel();
float getLeftVel();
float getRightVel();
void  resetForwardPos();
float getMaxDriveTemp();
void softStartTimerForward(float pwrInit, float pwrFinal, int durationMs);
void timerForward(float power, int durationMs,
                  float targetHeading = 0, float distanceLimit = 0);
void timerRotate(float power, int durationMs);
void posForwardAbs(float power, float target,
                   float targetHeading = 0);
void posForwardRel(float power, float target,
                   float targetHeading = 0);
void angleRotateAbs(float power, float target);
void pidForwardRel(float target, float errorTolerance = 2.0f);
void pidForwardAbs(float target, float errorTolerance = 2.0f);
void pidForwardAbs(float target, float kp, float ki, float kd, float errorTolerance = 2.0f);
void pidRotateRel(float target, float errorTolerance = 2.0f);
void pidRotateAbs(float target, float errorTolerance = 2.0f);
void pidRotateAbs(float target, float kp, float ki, float kd, float errorTolerance = 2.0f);
void posCurve(float leftPwr, float rightPwr, float target, bool mirror = true);
void PIDPosCurveAbs(float leftTarget, float rightTarget, float tolerance = 2.0f);
void PIDPosCurveAbs(float leftTarget, float rightTarget, float kp, float ki, float kd, float tolerance = 2.0f);
void driveWithHeading(float power, float targetHeading);
float gpsCalTheta(float dx, float dy, int direct);
void  gpsCalTargetPara(float xTarget, float yTarget, int direct,
                       float &outHeading, float &outDistance);
void  gpsMove(float xTarget, float yTarget, int direct,
              float maxPower, const char* mode);
void  gpsAim(float xTarget, float yTarget, int direct,
             float headingOffset = 0);
}

#endif  