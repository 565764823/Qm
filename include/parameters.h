#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <cstdint>


namespace Physics {
inline constexpr float kWheelDiameter      = 3.25f;   // 驱动轮直径
inline constexpr float kWheelCircumference = kWheelDiameter * 3.141592653589793;  // 驱动轮周长
inline constexpr float kWheelCircMM        = kWheelCircumference * 25.4f;          // 驱动轮周长
inline constexpr float kChassisRadius      = 7.0f;    // 底盘半径，用于转弯运动学
inline constexpr float kTrackWidth         = 12.0f;   // 轮距，左右轮中心间距
inline constexpr float kAxleBase           = 10.0f;   // 轴距，前后轮中心间距
inline constexpr float kMaxSpeedPercent    = 100.0f;  // 最大速度百分比上限
inline constexpr float kNominalVoltage     = 12.0f;   // 电机标称电压
inline constexpr float kMmPerInch          = 25.4f;   // 毫米/英寸换算常数
}


namespace Encoder {
inline constexpr float kTicksPerRev  = 900.0f;   // 编码器每转脉冲数
inline constexpr float kTicksPerInch = kTicksPerRev / Physics::kWheelCircumference;  // 每英寸行程
inline constexpr float kDegPerRev    = 360.0f;   // 每转角度
}

namespace Thermal {
inline constexpr double kWarning  = 50.0;  
inline constexpr double kThrottle = 60.0;  
inline constexpr double kShutdown = 70.0;  


inline double powerScale(double temp) {
    if (temp < kWarning)  return 1.0;   
    if (temp < kThrottle) return 0.75;  // 75% 功率
    if (temp < kShutdown) return 0.50;  // 节流~停机之间，50% 功率
    return 0.0;                          // 超过停机温度，0% 功率
}
}


inline constexpr int kDriveMaxCombinedPower = 100;  // 驱动系统最大综合功率

namespace Chassis {
inline constexpr float kDefaultVelLimit     = 80.0f;   // 默认速度限制
inline constexpr float kVelToDutyScale      = 1.0f;    // 速度到占空比的缩放系数
inline constexpr int   kVelSoftStartMinMs   = 50;      // 软启动最小持续时间，防止急加速
}


namespace Tuning {
inline constexpr float kKpStep          = 0.05f;    // Kp 粗调步长
inline constexpr float kKiStep          = 0.01f;    // Ki 粗调步长
inline constexpr float kKdStep          = 0.50f;    // Kd 粗调步长
inline constexpr float kKpStepFine      = 0.01f;    // Kp 微调步长
inline constexpr float kKiStepFine      = 0.005f;   // Ki 微调步长
inline constexpr float kKdStepFine      = 0.10f;    // Kd 微调步长
inline constexpr float kTestForwardMm   = 1200.0f;  // 前进测试距离
inline constexpr float kTestRotateDeg   = 90.0f;    // 旋转测试角度
inline constexpr float kTestErrorTol    = 5.0f;     // 测试误差容限
}

namespace GPS {
inline constexpr float kDefaultMaxPower     = 60.0f;   // 导航默认最大功率
inline constexpr float kApproachRatio       = 0.70f;   // 接近目标时的功率比例，控制减速
inline constexpr float kAimTolerance        = 2.0f;    // 瞄准容差
inline constexpr float kHeadingOffset       = -90.0f;  // 航向角偏移
inline constexpr int   kSoftStartMsPerPower = 2;       // 软启动时间
}

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

struct PIDConfig {
    float kp, ki, kd;          // PID 比例/积分/微分增益
    float iMax, iRange;        // 积分上限、积分生效范围
    float errorTol, dTol;      // 误差容限、微分容限
    float jumpTime;            // 目标跳变检测时间
};


namespace ChassisPID {


inline constexpr float kForwardKp = 0.30f;   // 前进 Kp
inline constexpr float kForwardKi = 0.05f;   // 前进 Ki
inline constexpr float kForwardKd = 3.25f;   // 前进 Kd

inline constexpr float kFwdIMax       = 30.0f;   // 前进积分上限
inline constexpr float kFwdIRange     = 50.0f;   // 前进积分生效范围
inline constexpr float kFwdErrorTol   = 5.0f;    // 前进误差容限
inline constexpr float kFwdDTol       = 10.0f;   // 前进微分容限
inline constexpr float kFwdJumpTime   = 150.0f;  // 前进口标跳变检测时间
inline constexpr float kFwdOutputLimit = 70.0f;  // 前进输出限幅

inline constexpr float kRotateKp = 1.20f;   // 旋转 Kp
inline constexpr float kRotateKi = 0.20f;   // 旋转 Ki
inline constexpr float kRotateKd = 12.0f;   // 旋转 Kd

inline constexpr float kRotIMax       = 20.0f;   // 旋转积分上限
inline constexpr float kRotIRange     = 30.0f;   // 旋转积分生效范围
inline constexpr float kRotErrorTol   = 2.0f;    // 旋转误差容限
inline constexpr float kRotDTol       = 5.0f;    // 旋转微分容限
inline constexpr float kRotJumpTime   = 150.0f;  // 旋转目标跳变检测时间
inline constexpr float kRotOutputLimit = 60.0f;  // 旋转输出限幅

inline constexpr float kCurveKp = kForwardKp;   // 弧线 Kp
inline constexpr float kCurveKi = kForwardKi;   // 弧线 Ki
inline constexpr float kCurveKd = kForwardKd;   // 弧线

inline constexpr float kCurveIMax       = 30.0f;   // 弧线积分上限
inline constexpr float kCurveIRange     = 50.0f;   // 弧线积分生效范围
inline constexpr float kCurveErrorTol   = 5.0f;    // 弧线误差容限
inline constexpr float kCurveDTol       = 10.0f;   // 弧线微分容限
inline constexpr float kCurveJumpTime   = 150.0f;  // 弧线目标跳变检测时
inline constexpr float kCurveOutputLimit = 70.0f;  // 弧线输出限幅

inline constexpr PIDConfig kFwdConfig{kForwardKp, kForwardKi, kForwardKd,
                                      kFwdIMax, kFwdIRange, kFwdErrorTol,
                                      kFwdDTol, kFwdJumpTime};   // 前进 PID 配置
inline constexpr PIDConfig kRotConfig{kRotateKp, kRotateKi, kRotateKd,
                                      kRotIMax, kRotIRange, kRotErrorTol,
                                      kRotDTol, kRotJumpTime};   // 旋转 PID 配置
inline constexpr PIDConfig kCurveConfig{kCurveKp, kCurveKi, kCurveKd,
                                        kCurveIMax, kCurveIRange, kCurveErrorTol,
                                        kCurveDTol, kCurveJumpTime};  // 弧线 PID 配置

}

namespace AutoPID {


inline constexpr float kFwdKp = 0.30f;   // 前进 Kp
inline constexpr float kFwdKi = 0.05f;   // 前进 Ki
inline constexpr float kFwdKd = 3.25f;   // 前进 Kd

inline constexpr float kFwdIMax     = 30.0f;   // 前进积分上限
inline constexpr float kFwdIRange   = 50.0f;   // 前进积分生效范围
inline constexpr float kFwdErrorTol = 5.0f;    // 前进误差容限
inline constexpr float kFwdDTol     = 10.0f;   // 前进微分容限
inline constexpr float kFwdJumpTime = 150.0f;  // 前进口标跳变检测时间


inline constexpr float kRotKp = 1.20f;   // 旋转 Kp
inline constexpr float kRotKi = 0.20f;   // 旋转 Ki
inline constexpr float kRotKd = 12.0f;   // 旋转 Kd

inline constexpr float kRotIMax     = 20.0f;   // 旋转积分上限
inline constexpr float kRotIRange   = 30.0f;   // 旋转积分生效范围
inline constexpr float kRotErrorTol = 2.0f;    // 旋转误差容限
inline constexpr float kRotDTol     = 5.0f;    // 旋转微分容限
inline constexpr float kRotJumpTime = 150.0f;  // 旋转目标跳变检测时间

inline constexpr float kCurveKp = kFwdKp;   // 弧线 Kp
inline constexpr float kCurveKi = kFwdKi;   // 弧线 Ki
inline constexpr float kCurveKd = kFwdKd;   // 弧线 Kd

inline constexpr float kCurveIMax     = 30.0f;   // 弧线积分上限
inline constexpr float kCurveIRange   = 50.0f;   // 弧线积分生效范围
inline constexpr float kCurveErrorTol = 5.0f;    // 弧线误差容限
inline constexpr float kCurveDTol     = 10.0f;   // 弧线微分容限
inline constexpr float kCurveJumpTime = 150.0f;  // 弧线目标跳变检测时间

// 预组装的 PIDConfig 实例
inline constexpr PIDConfig kFwd{kFwdKp, kFwdKi, kFwdKd,
                                kFwdIMax, kFwdIRange, kFwdErrorTol,
                                kFwdDTol, kFwdJumpTime};  // 前进 PID 配置
inline constexpr PIDConfig kRot{kRotKp, kRotKi, kRotKd,
                                kRotIMax, kRotIRange, kRotErrorTol,
                                kRotDTol, kRotJumpTime};  // 旋转 PID 配置
inline constexpr PIDConfig kCurve{kCurveKp, kCurveKi, kCurveKd,
                                  kCurveIMax, kCurveIRange, kCurveErrorTol,
                                  kCurveDTol, kCurveJumpTime};  // 弧线 PID 配置

// 预设的运动速度档位
inline constexpr PIDConfig kFwdSlow{0.20f, 0.03f, 2.00f, 30, 50, 5, 10, 150};  // 前进慢速档 PID
inline constexpr PIDConfig kFwdFast{0.50f, 0.08f, 5.00f, 30, 50, 5, 10, 150};  // 前进快速档 PID

}

//==============================================================================
// Motion — 运动规划参数（梯形速度/减速控制）
//==============================================================================
namespace Motion {
// --- 位置（前进/后退）运动 ---
inline constexpr float kPosRemainingThreshold = 5.0f;    // 到位判定阈值
inline constexpr float kPosDecelDistance      = 200.0f;  // 减速距离
inline constexpr float kPosMinPowerRatio      = 0.15f;   // 最小功率比例
inline constexpr float kPosMaxPowerRatio      = 1.0f;    // 最大功率比例

// --- 角度旋转运动 ---
inline constexpr float kAngleRemainingThreshold = 3.0f;   // 到位判定阈值
inline constexpr float kAngleDecelAngle         = 45.0f;  // 减速起始角度
inline constexpr float kAngleMinPowerRatio      = 0.15f;  // 最小功率比例
inline constexpr float kAngleMaxPowerRatio      = 1.0f;   // 最大功率比例

// --- 航向修正 ---
inline constexpr float kHeadingCorrectionGain = 0.6f;    // 航向修正
inline constexpr float kHeadingCorrectionMax  = 40.0f;   // 航向修正输出上限
inline constexpr float kHeadingCorrDeadband   = 1.0f;    // 航向修正死区
}

namespace Auto {
inline constexpr int kRouteCount = 6;  // 自动路线总数
}

inline constexpr float kIMUScale = 3568.0f;  // IMU 原始值到角度的换算系数

inline constexpr float kIMUHeadingConversion = 3600.0f;  // IMU 航向转换系数
inline constexpr float kIMUDriftThreshold    = 0.5f;     // IMU 漂移检测阈值
inline constexpr float kIMUDeviationThreshold = 10.0f;   // IMU 偏差检测阈值


inline constexpr float kOdometryWheelDiameter = 2.75f;                              // 里程计轮直径
inline constexpr float kOdometryTicksPerRev   = 360.0f;                             // 里程计编码器每转脉冲数
inline constexpr float kOdometryWheelCirc     = kOdometryWheelDiameter * 3.141592653589793f;  // 里程计轮周长
inline constexpr float kOdometryTicksPerInch  = kOdometryTicksPerRev / kOdometryWheelCirc;     

namespace Odometry {
inline constexpr float kFwdWheelOffsetX = 0.0f;     // 前进跟踪轮 X 方向安装偏移
inline constexpr float kFwdWheelOffsetY = 0.0f;     // 前进跟踪轮 Y 方向安装偏移
inline constexpr float kStrWheelOffsetX = 0.0f;     // 侧向跟踪轮 X 方向安装偏移
inline constexpr float kStrWheelOffsetY = 0.0f;     // 侧向跟踪轮 Y 方向安装偏移

inline constexpr float kDriftCorrGain    = 0.001f;  // IMU 漂移修正增益，控制修正速度
inline constexpr float kStuckThreshold   = 0.02f;   
inline constexpr int   kStuckFrames      = 5;       // 卡滞判定帧数

inline constexpr float kArcEpsilonRad = 0.001f;     // 弧线计算

inline constexpr float kEmaAlpha = 0.3f;            

inline constexpr float kStuckRecoveryThreshold = 0.05f;  

inline constexpr int kStuckCooldownFrames = 10;     
}

inline constexpr float kRedHue         = 15.0f;   // 红色色相中心值
inline constexpr float kBlueHue        = 215.0f;  // 蓝色色相中心值
inline constexpr float kYellowHue      = 40.0f;   // 黄色色相中心值
inline constexpr float kColorTolerance = 20.0f;   // 颜色识别容差，色相值 ± 容差范围内认为匹配


inline constexpr float kMovementLowerLimit = 3.0f;   // 移动下限阈值，低于此距离不做运动
inline constexpr float kADSFactor          = 0.4f;   // ADS


namespace Timing {
inline constexpr int kInitPreflightDelay    = 300;   // 初始化预检延迟，上电后等待传感器就绪
inline constexpr int kIMUCalibPollInterval  = 10;    // IMU 校准轮询间隔
inline constexpr int kInitArmedDelay        = 400;   // 预检完成到进入待命状态的延迟
inline constexpr int kInitReadyDelay        = 600;   // 待命到就绪的延迟，确保所有系统就绪
inline constexpr int kUserLoopInterval      = 10;    // 用户控制主循环间隔
inline constexpr int kControllerPollInterval = 10;   // 手柄轮询间隔
inline constexpr int kOdometryLoopInterval  = 10;    // 里程计计算循环
inline constexpr int kIMUInspectionInterval = 100;   // IMU 状态检测间隔
}


namespace Dashboard {
inline constexpr int kRefreshTicks           = 10;    // 面板刷新周期
inline constexpr int kDriftLatchCountdown    = 5;     // 漂移锁存倒计时
inline constexpr int kIMUInspectIntervalTicks = 300;  // IMU 检测间隔
}

#endif
