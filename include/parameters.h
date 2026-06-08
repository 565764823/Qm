#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <cstdint>

namespace Physics {
inline constexpr float kWheelDiameter      = 3.25f;   
inline constexpr float kWheelCircumference = kWheelDiameter * 3.141592653589793f;
inline constexpr float kWheelCircMM        = kWheelCircumference * 25.4f;
inline constexpr float kChassisRadius      = 7.0f;    
inline constexpr float kTrackWidth         = 12.0f;   
inline constexpr float kAxleBase           = 10.0f;   
inline constexpr float kMaxSpeedPercent    = 100.0f;  
inline constexpr float kNominalVoltage     = 12.0f;   
inline constexpr float kMmPerInch          = 25.4f;   
}  

namespace Encoder {
inline constexpr float kTicksPerRev  = 900.0f;   
inline constexpr float kTicksPerInch = kTicksPerRev / Physics::kWheelCircumference;
inline constexpr float kDegPerRev    = 360.0f;   
}  

namespace Thermal {
inline constexpr double kWarning  = 50.0;   
inline constexpr double kThrottle = 60.0;   
inline constexpr double kShutdown = 70.0;   

inline double powerScale(double temp) {
    if (temp < kWarning)  return 1.0;
    if (temp < kThrottle) return 0.75;
    if (temp < kShutdown) return 0.50;
    return 0.0;
}
}  

inline constexpr int kDriveMaxCombinedPower = 100;   

namespace Ctrl {
inline constexpr float kJoystickDeadzone = 1.0f;     
inline constexpr float kTurnSensitivity  = 0.80f;   
inline constexpr float kSpeedSensitivity = 0.80f;    
}  

namespace PIDDefaults {
inline constexpr float kErrorTol = 1.0f;
inline constexpr float kDTol     = 10.0f;
inline constexpr float kIMax     = 20.0f;
inline constexpr float kIRange   = 50.0f;
inline constexpr float kJumpTime = 50.0f;
}  


namespace ChassisPID {

#ifdef ROBOT_01
inline constexpr float kForwardKp = 0.30f;
inline constexpr float kForwardKi = 0.05f;
inline constexpr float kForwardKd = 3.25f;
#endif
#ifdef ROBOT_02
inline constexpr float kForwardKp = 0.30f;
inline constexpr float kForwardKi = 0.05f;
inline constexpr float kForwardKd = 3.25f;
#endif

inline constexpr float kFwdIMax       = 30.0f;
inline constexpr float kFwdIRange     = 50.0f;
inline constexpr float kFwdErrorTol   = 5.0f;
inline constexpr float kFwdDTol       = 10.0f;
inline constexpr float kFwdJumpTime   = 150.0f;
inline constexpr float kFwdOutputLimit = 70.0f;   

#ifdef ROBOT_01
inline constexpr float kRotateKp = 1.20f;
inline constexpr float kRotateKi = 0.20f;
inline constexpr float kRotateKd = 12.0f;
#endif
#ifdef ROBOT_02
inline constexpr float kRotateKp = 1.20f;
inline constexpr float kRotateKi = 0.20f;
inline constexpr float kRotateKd = 12.0f;
#endif

inline constexpr float kRotIMax       = 20.0f;
inline constexpr float kRotIRange     = 30.0f;
inline constexpr float kRotErrorTol   = 2.0f;
inline constexpr float kRotDTol       = 5.0f;
inline constexpr float kRotJumpTime   = 150.0f;
inline constexpr float kRotOutputLimit = 60.0f;   

inline constexpr float kCurveKp = kForwardKp;
inline constexpr float kCurveKi = kForwardKi;
inline constexpr float kCurveKd = kForwardKd;

inline constexpr float kCurveIMax       = 30.0f;
inline constexpr float kCurveIRange     = 50.0f;
inline constexpr float kCurveErrorTol   = 5.0f;
inline constexpr float kCurveDTol       = 10.0f;
inline constexpr float kCurveJumpTime   = 150.0f;
inline constexpr float kCurveOutputLimit = 70.0f;  

}  

namespace Motion {
inline constexpr float kPosRemainingThreshold = 5.0f;   
inline constexpr float kPosDecelDistance      = 200.0f;  
inline constexpr float kPosMinPowerRatio      = 0.15f;   
inline constexpr float kPosMaxPowerRatio      = 1.0f;    

inline constexpr float kAngleRemainingThreshold = 3.0f;  
inline constexpr float kAngleDecelAngle         = 45.0f; 
inline constexpr float kAngleMinPowerRatio      = 0.15f; 
inline constexpr float kAngleMaxPowerRatio      = 1.0f;  

inline constexpr float kHeadingCorrectionGain = 0.6f;    
inline constexpr float kHeadingCorrectionMax  = 40.0f;   
}  

#ifdef ROBOT_01
inline constexpr float kIMUScale = 3568.0f;   
#endif
#ifdef ROBOT_02
inline constexpr float kIMUScale = 3568.0f;
#endif

inline constexpr float kIMUHeadingConversion = 3600.0f;  
inline constexpr float kIMUDriftThreshold    = 0.5f;     
inline constexpr float kIMUDeviationThreshold = 10.0f;   

inline constexpr float kOdometryWheelDiameter = 2.75f;   
inline constexpr float kOdometryTicksPerRev   = 360.0f;  
inline constexpr float kOdometryWheelCirc     = kOdometryWheelDiameter * 3.141592653589793f;
inline constexpr float kOdometryTicksPerInch  = kOdometryTicksPerRev / kOdometryWheelCirc;

inline constexpr float kRedHue         = 15.0f;
inline constexpr float kBlueHue        = 215.0f;
inline constexpr float kYellowHue      = 40.0f;
inline constexpr float kColorTolerance = 20.0f;

inline constexpr float kMovementLowerLimit = 3.0f;
inline constexpr float kADSFactor          = 0.4f;

namespace Timing {
inline constexpr int kInitPreflightDelay    = 300;   
inline constexpr int kIMUCalibPollInterval  = 10;    
inline constexpr int kInitArmedDelay        = 400;   
inline constexpr int kInitReadyDelay        = 600;   
inline constexpr int kUserLoopInterval      = 10;    
inline constexpr int kControllerPollInterval = 10;   
inline constexpr int kOdometryLoopInterval  = 10;    
inline constexpr int kIMUInspectionInterval = 100;   
}  

namespace Dashboard {
inline constexpr int kRefreshTicks        = 10;   
inline constexpr int kDriftLatchCountdown = 5;   
}  

#endif  
