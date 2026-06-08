#ifndef TIMER_H_
#define TIMER_H_

#include "vex.h"

extern vex::brain Brain;

class MyTimer {
private:
    double startTime;  

public:
    MyTimer() : startTime(Brain.Timer.value()) {}
    explicit MyTimer(float offsetMs)
        : startTime(Brain.Timer.value() + offsetMs / 1000.0f) {}
    inline void reset() {
        startTime = Brain.Timer.value();
    }
    inline int getTime() const {
        return static_cast<int>((Brain.Timer.value() - startTime) * 1000.0f);
    }
    inline float getTimeSec() const {
        return static_cast<float>(Brain.Timer.value() - startTime);
    }
};

#endif  
