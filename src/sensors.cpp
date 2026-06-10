#include "sensors.h"
#include "config.h"
#include "parameters.h"
#include <thread>

using namespace vex;

float getHeading() {
    return IMU.rotation() / kIMUScale * kIMUHeadingConversion;
}

void imu_inspection(float _rotation) {
    float temp = getHeading();
    this_thread::sleep_for(Timing::kIMUInspectionInterval);
    if (fabs(getHeading() - temp) > kIMUDriftThreshold) {
        IMU.calibrate();
        while (IMU.isCalibrating()) {
            this_thread::sleep_for(Timing::kIMUInspectionInterval);
        }
        IMU.setRotation(_rotation, degrees);
    }
    if (fabs(getHeading() - _rotation) > kIMUDeviationThreshold)
        IMU.setRotation(_rotation, degrees);
}
