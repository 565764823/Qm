#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "config.h"
#include "parameters.h"

extern PIDConfig autoFwd;
extern PIDConfig autoRot;
extern PIDConfig autoCurve;
extern int auton_strategy;

extern bool g_auto_active;

void auto_resetPID();

void auto_run();
int  auto_cycle();
const char* auto_name();

void auto_notify_start();
bool auto_is_active();

#endif
