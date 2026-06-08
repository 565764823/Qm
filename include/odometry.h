#ifndef ODOMETRY_H_
#define ODOMETRY_H_
namespace odometry {
void init(float x = 0.0f, float y = 0.0f);
float rawX();
float rawY();
float globalX();
float globalY();
float heading();
void setPosition(float x, float y);
bool calibrated();
void updateLoop();

}  
#endif  
