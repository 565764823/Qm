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

bool auto_is_active();

#endif
