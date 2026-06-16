
#ifndef CONFIG_H_
#define CONFIG_H_

#include "vex.h"
#include <cstdint>

#if !defined(ROBOT_01) && !defined(ROBOT_02)
  #define ROBOT_01
#endif

#include "parameters.h"

namespace Port {

#ifdef ROBOT_01
  inline constexpr int32_t kDriveAuxL = 0;
  inline constexpr int32_t kDriveFL    = 1;
  inline constexpr int32_t kDriveFR    = 6;
  inline constexpr int32_t kDriveAuxR = 8;
  inline constexpr int32_t kDriveBL    = 10;
  inline constexpr int32_t kDriveBR    = 18;
  inline constexpr int32_t kIMU       = 21;
  inline constexpr int32_t kOdomX     = 5;
  inline constexpr int32_t kOdomY     = 20;

#endif

#ifdef ROBOT_02
  inline constexpr int32_t kDriveAuxL = 1;
  inline constexpr int32_t kDriveFL    = 2;
  inline constexpr int32_t kDriveFR    = 7;
  inline constexpr int32_t kDriveAuxR = 9;
  inline constexpr int32_t kDriveBL    = 11;
  inline constexpr int32_t kDriveBR    = 19;
  inline constexpr int32_t kIMU       = 21;
  inline constexpr int32_t kOdomX     = 5;
  inline constexpr int32_t kOdomY     = 20;

#endif

} 

namespace Gear {
  inline const vex::gearSetting kDrive = vex::ratio18_1;
}  


extern vex::brain       Brain;
extern vex::controller   Controller;
extern vex::motor Motor_AuxL;
extern vex::motor Motor_FL;
extern vex::motor Motor_FR;
extern vex::motor Motor_BL;
extern vex::motor Motor_BR;
extern vex::motor Motor_AuxR;

extern vex::inertial IMU;
extern vex::rotation Odometer[2];

void initRobot();
void emergencyStop();

#endif  // CONFIG_H_
