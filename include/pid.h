#ifndef PID_H_
#define PID_H_

#include "math-utils.h"
#include "timer.h"
#include "parameters.h"
#include <cmath>

class PID {
private:
    float errorCrt  = 0;    
    float errorPrev = 0;   
    float errorDev  = 0;   
    float errorInt  = 0;  
    float P = 0, I = 0, D = 0;
    float kp = 0, ki = 0, kd = 0;
    float target   = 0;      
    float errorTol = PIDDefaults::kErrorTol;
    float DTol     = PIDDefaults::kDTol;
    float IMax     = PIDDefaults::kIMax;
    float IRange   = PIDDefaults::kIRange;
    float output   = 0;
    float jumpTime = PIDDefaults::kJumpTime;    
    bool firstTime     = true;  
    bool arrived       = false; 
    bool timer_started = false;  

    MyTimer settleTimer;       

public:
    PID() { settleTimer.reset(); }
    PID(const PIDConfig& cfg)
        : kp(cfg.kp), ki(cfg.ki), kd(cfg.kd)
        , IMax(cfg.iMax), IRange(cfg.iRange), errorTol(cfg.errorTol)
        , DTol(cfg.dTol), jumpTime(cfg.jumpTime)
    { settleTimer.reset(); }

    inline void setFirstTime() {
        firstTime = true;
        arrived   = false;
        settleTimer.reset();
    }
    inline void setCoefficient(float _kp, float _ki, float _kd) {
        kp = _kp;  ki = _ki;  kd = _kd;
    }
    inline float getKp() const { return kp; }
    inline float getKi() const { return ki; }
    inline float getKd() const { return kd; }

    inline void setTarget(float _target)           { target = _target; }
    inline void setIMax(float _IMax)                { IMax = _IMax; }
    inline void setIRange(float _IRange)            { IRange = _IRange; }
    inline void setErrorTolerance(float _errorTol)   { errorTol = _errorTol; }
    inline void setDTolerance(float _DTol)          { DTol = _DTol; }
    inline void setJumpTime(float _jumpTime)        { jumpTime = _jumpTime; }

    inline bool  targetArrived() const { return arrived; }
    inline bool  timerStarted()  const { return timer_started; }
    inline float getOutput()     const { return output; }
    inline float getError()      const { return errorCrt; }
    inline float getP()          const { return P; }
    inline float getI()          const { return I; }
    inline float getD()          const { return D; }

    inline void update(float input) {
        errorCrt = target - input;
        P = kp * errorCrt;

 
        if (firstTime) {
            firstTime  = false;
            errorPrev  = errorCrt;
            errorInt   = 0;
        }

        errorDev = errorCrt - errorPrev;
        errorPrev = errorCrt;
        D = kd * errorDev;


        if (std::abs(P) >= IRange) {
            errorInt = 0;
        } else {
            
            errorInt += errorCrt;
    
            if (std::abs(errorInt) * ki > IMax)
                errorInt = math::sign(errorInt) * IMax / ki;
        }
        if (math::sign(errorInt) != math::sign(errorCrt) ||
            std::abs(errorCrt) <= errorTol)
            errorInt = 0;

        I = ki * errorInt;

        if (std::abs(errorCrt) <= errorTol && std::abs(D) <= DTol) {
            timer_started = true;
            if (settleTimer.getTime() >= static_cast<int>(jumpTime))
                arrived = true;
        } else {
            settleTimer.reset();
            timer_started = false;
        }
        output = P + I + D;
    }
};

#endif  