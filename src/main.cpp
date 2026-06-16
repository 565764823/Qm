#include "config.h"
#include "chassis.h"
#include "sensors.h"
#include "odometry.h"
#include "parameters.h"
#include "autonomous.h"
#include "controller.h"
#include "adjustment.h"
#include <cstdlib>
#include <thread>

using namespace vex;
brain       Brain;
controller  Controller = controller(primary);

motor Motor_AuxL = motor(Port::kDriveAuxL, Gear::kDrive, false);
motor Motor_FL   = motor(Port::kDriveFL,   Gear::kDrive, true);
motor Motor_FR   = motor(Port::kDriveFR,   Gear::kDrive, false);
motor Motor_BL   = motor(Port::kDriveBL,   Gear::kDrive, false);
motor Motor_BR   = motor(Port::kDriveBR,   Gear::kDrive, true);
motor Motor_AuxR = motor(Port::kDriveAuxR, Gear::kDrive, true);

inertial IMU = inertial(Port::kIMU);
rotation Odometer[2] = {
    rotation(Port::kOdomX, true),
    rotation(Port::kOdomY, false)
};

competition Competition;

void initRobot() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono20);

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("  === X-DRIVE v3 ===");
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("  [1/3] PREFLIGHT");
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("  Scanning devices...");
    this_thread::sleep_for(Timing::kInitPreflightDelay);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("  [2/3] IMU CALIB  ");
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("  Calibrating...      ");
    IMU.startCalibration();
    while (IMU.isCalibrating()) {
        this_thread::sleep_for(Timing::kIMUCalibPollInterval);
    }
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("  IMU Ready!          ");

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("  [3/3] ARMED      ");
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("  All systems: GO     ");
    this_thread::sleep_for(Timing::kInitArmedDelay);

    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono40);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("  READY");
    this_thread::sleep_for(Timing::kInitReadyDelay);
}
void emergencyStop() {
    chassis::brakeAll();

    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono40);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("  EMERGENCY");
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("  STOP");
}

void usercontrol(void);  

void competitionAuton(void) {
    odometry::setPositionMm(0, 0);
    auto_run();
}

void competitionDriver(void) {
    usercontrol();
}

void usercontrol(void) {
    Brain.Screen.clearScreen();

    static float  lastImuHeading    = getHeading();
    static bool   imuDriftWarn      = false;
    static int    imuDriftCountdown = 0;

    bool running = true;

    while (running) {
        defineController();

        if (btnA && !lastA) {
            chassis::brakeAll();
            running = false;
            continue;
        }

        if (btnX && !lastX) {
            chassis::brakeAll();
            tuningForward();
            Brain.Screen.clearScreen();
        }

        if (btnY && !lastY) {
            chassis::brakeAll();
            tuningRotate();
            Brain.Screen.clearScreen();
        }

        int fwd  = (std::abs(joyA3) < Ctrl::kJoystickDeadzone) ? 0 : joyA3;
        int turn = (std::abs(joyA1) < Ctrl::kJoystickDeadzone) ? 0 : joyA1;

        float temp = chassis::getMaxDriveTemp();
        float tempScale = static_cast<float>(Thermal::powerScale(temp));
        fwd  = static_cast<int>(fwd  * tempScale);
        turn = static_cast<int>(turn * tempScale);
        turn = static_cast<int>(turn * Ctrl::kTurnSensitivity);
        fwd  = static_cast<int>(fwd  * Ctrl::kSpeedSensitivity);
        chassis::arcadeDrive(fwd, turn);

        if (temp >= Thermal::kWarning) {
            Controller.rumble(". . . .");
        }

        static int dashTick = 0;
        if (++dashTick >= Dashboard::kRefreshTicks) {
            dashTick = 0;
            Brain.Screen.setFont(mono15);

            float curHeading = getHeading();

            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("=== DASHBOARD ===");

            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("POS %+8.0fmm  HDG %+6.1f%c",
                               chassis::getForwardPos(), curHeading, 0xB0);

            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("ODO  %+6.2f  %+6.2f in",
                               odometry::globalX(), odometry::globalY());

            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("ROB  %+6.2f  %+6.2f in  %-6s",
                               odometry::robotGlobalX(), odometry::robotGlobalY(),
                               odometry::isStuck() ? "STUCK!" : "OK");

            Brain.Screen.setCursor(5, 1);
            Brain.Screen.print("--- DRIVE ---");

            Brain.Screen.setCursor(6, 1);
            Brain.Screen.print("PWR F:%+4d  T:%+4d  MAX %3.0fC",
                               fwd, turn, temp);

            const char* thermalStatus;
            if (temp > Thermal::kShutdown)      thermalStatus = "STOP!";
            else if (temp > Thermal::kThrottle) thermalStatus = "HOT";
            else if (temp > Thermal::kWarning)  thermalStatus = "WARM";
            else                                thermalStatus = "OK";
            Brain.Screen.print(" %-5s", thermalStatus);

            Brain.Screen.setCursor(7, 1);
            Brain.Screen.print("FL:%3.0f FR:%3.0f BL:%3.0f BR:%3.0f",
                               Motor_FL.temperature(celsius),
                               Motor_FR.temperature(celsius),
                               Motor_BL.temperature(celsius),
                               Motor_BR.temperature(celsius));

            Brain.Screen.setCursor(8, 1);
            Brain.Screen.print("BATT %3d%%  IMU ",
                               Brain.Battery.capacity());

            if (IMU.isCalibrating())
                Brain.Screen.print("%-8s", "CALIB...");
            else if (imuDriftWarn)
                Brain.Screen.print("%-8s", "DRIFT!");
            else
                Brain.Screen.print("%-8s", "OK");

            if (fwd == 0 && turn == 0) {
                float drift = fabs(curHeading - lastImuHeading);
                if (drift > kIMUDriftThreshold * Dashboard::kRefreshTicks) {
                    imuDriftCountdown = Dashboard::kDriftLatchCountdown;
                    imuDriftWarn = true;
                }
            } else {
                imuDriftWarn = false;
            }
            if (imuDriftCountdown > 0) {
                imuDriftCountdown--;
                if (imuDriftCountdown == 0) imuDriftWarn = false;
            }
            lastImuHeading = curHeading;

            Brain.Screen.setCursor(11, 1);
            Brain.Screen.print("[A]退出  [X]前进PID  [Y]旋转PID");
        }

        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}
int main() {
    initRobot();

    odometry::init(0.0f, 0.0f);
    thread odomThread(odometry::updateLoop);

    Competition.autonomous(competitionAuton);
    Competition.drivercontrol(competitionDriver);

    while (true) {

        Brain.Screen.clearScreen();
        Brain.Screen.setFont(mono20);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("  AUTO: 等待选择...");
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("  ← → 切换  A 启动  B 手动");

        bool auto_done = false;
        static int imuCheckTick = 0;
        while (!auto_done) {
            defineController();

            if (btnRight && !lastRight)  auto_cycle();
            if (btnLeft  && !lastLeft)   auto_cycle();

            Brain.Screen.setCursor(7, 1);
            Brain.Screen.print("  路线[%d]: %s", auton_strategy, auto_name());

            if (++imuCheckTick >= Dashboard::kIMUInspectIntervalTicks) {
                imuCheckTick = 0;
                imu_inspection(getHeading());
            }

            if (btnB && !lastB) { auto_done = true; }
            if (btnA && !lastA) {
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("  AUTO RUNNING...");
                Brain.Screen.setCursor(5, 1);
                Brain.Screen.print("  B: 中断 → 手动");
                auto_run();
                auto_done = true;
            }

            this_thread::sleep_for(Timing::kUserLoopInterval);
        }

        usercontrol();
      
    }
}
