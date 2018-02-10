#ifndef SRC_SHOOTER_H_
#define SRC_SHOOTER_H_
#include "WPILib.h"
#include "WPILib_auxiliary.h"
#define PI 3.14159265359
#define G 9.81
#define RADIUS 1
#define K_sp 1
#define K_si 0
#define K_sd 0

class Shooter
{
private:
	Timer *timer;
	double curr_t =0.0;
	double old_t =0.0;
	double del_t =0.0;
	void get_delta_time();
	double iCount;
	double dOld;
	double angPerVolt;
	double getTargetAngle(double v,double y, double x);
	double getVelocity(double deltaAngle);
	double getAngle(AnalogPotentiometer *pot);
public:
	Shooter();
	void aim_shooter(Encoder *encode, AnalogPotentiometer *pot,CANTalon *M_shooter);
};

#endif /* SRC_IMU_H_ */
