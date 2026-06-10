#include "config.h"
#include "chassis.h"
#include "sensors.h"
#include "odometry.h"
#include "parameters.h"
#include "autonomous.h"
#include "controller.h"
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
void usercontrol(void) {
    Brain.Screen.clearScreen();

    static float  lastImuHeading    = getHeading();
    static bool   imuDriftWarn      = false;
    static int    imuDriftCountdown = 0;

    bool running   = true;
    bool lastA_man = false;

    while (running) {
        bool currA = Controller.ButtonA.pressing();
        if (currA && !lastA_man) {
            chassis::brakeAll();
            running = false;
            continue;
        }
        lastA_man = currA;

        int joyA3 = Controller.Axis3.position();
        int joyA1 = Controller.Axis1.position();

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
            Brain.Screen.print("ODO RAW X:%7.0f  Y:%7.0f deg",
                               odometry::rawX(), odometry::rawY());

            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("POS: %+8.0fmm  HDG: %+6.1f%c",
                               chassis::getForwardPos(), curHeading, 0xB0);

            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("X:%+7.2f  Y:%+7.2f in",
                               odometry::globalX(), odometry::globalY());

            Brain.Screen.setCursor(5, 1);
            Brain.Screen.print("PWR F:%+4d  T:%+4d  TEMP:%4.0fC",
                               fwd, turn, temp);

            Brain.Screen.setCursor(6, 1);
            Brain.Screen.print("BATT:%3d%%  IMU: ",
                               Brain.Battery.capacity());

            if (IMU.isCalibrating())
                Brain.Screen.print("CALIB...");
            else if (imuDriftWarn)
                Brain.Screen.print("DRIFT!");
            else
                Brain.Screen.print("OK   ");

            if (fwd == 0 && turn == 0) {
                float drift = fabs(curHeading - lastImuHeading);
                if (drift > kIMUDriftThreshold * Dashboard::kRefreshTicks) {
                    imuDriftCountdown = Dashboard::kDriftLatchCountdown;   /* latch warning for ~0.5 s */
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

            Brain.Screen.setCursor(8, 1);
            if (temp > Thermal::kShutdown)
                Brain.Screen.print("THERMAL: SHUTDOWN!");
            else if (temp > Thermal::kThrottle)
                Brain.Screen.print("THERMAL: THROTTLED");
            else if (temp > Thermal::kWarning)
                Brain.Screen.print("THERMAL: WARM");
            else
                Brain.Screen.print("THERMAL: OK");

            Brain.Screen.setCursor(9, 1);
            Brain.Screen.print("A:切换自动              ");
        }

        this_thread::sleep_for(Timing::kUserLoopInterval);
    }
}
int main() {
    initRobot();

    odometry::init(0.0f, 0.0f);
    thread odomThread(odometry::updateLoop);

    while (true) {

        Brain.Screen.clearScreen();
        Brain.Screen.setFont(mono20);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("  AUTO: 等待选择...");
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("  ← → 切换  A 启动  B 手动");

        bool auto_done = false;
        while (!auto_done) {
            defineController();

            if (btnRight && !lastRight)  auto_cycle();
            if (btnLeft  && !lastLeft)   auto_cycle();

            Brain.Screen.setCursor(7, 1);
            Brain.Screen.print("  当前路线: %d", auton_strategy);

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

            this_thread::sleep_for(10);
        }

       
        usercontrol();
      
    }
}
