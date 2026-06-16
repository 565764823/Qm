#ifndef ODOMETRY_H_
#define ODOMETRY_H_
namespace odometry {

void init(float x = 0.0f, float y = 0.0f);
float rawX();
float rawY();
float globalX();
float globalY();
float globalXMm();
float globalYMm();
float robotGlobalX();
float robotGlobalY();
float robotGlobalXMm();
float robotGlobalYMm();
float heading();
void setPosition(float x, float y);
void setPositionMm(float xMm, float yMm);
bool calibrated();
bool isStuck();
void calibrate();
void updateLoop();
}
#endif
