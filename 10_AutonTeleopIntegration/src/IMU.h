#ifndef SRC_IMU_H_
#define SRC_IMU_H_
#include "WPILib.h"
#include "WPILib_auxiliary.h"


#define INCHES_TO_CENTIMETERS 2.54
#define RAD 3.14159 / 180.0

#define K_a0 0.5
#define K_a1 0.5

#define tau 0.45


class IMU
{
private:
	Timer *timer;

double i = 0.0;
	double curr_t =0.0;
	double old_t =0.0;


	double complementaryIMU(double accel_magnitude, double gyroAngle, int loop);
	void positionFilter(double ax, double ay, double ex, double ey);
	double kalmanIMU(double accelAngle, double gyroAngle);
	double modulo(double i, double j);
public:
	double theta;
	double accel_x;
	double accel_y;
	double velocity_x;
	double velocity_y;
	double position_x;
	double position_y;

	double test_positionX;
	double test_positionY;
	double position_r;

	IMU();
	void Localization(AHRS *nav);
	void get_delta_time();
	double del_t =0.0;
	void Reset(AHRS *nav);
	void ResetPos();
};

#endif /* SRC_IMU_H_ */
