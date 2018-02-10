#ifndef SRC_IMU_H_
#define SRC_IMU_H_
#include "WPILib.h"
#include "WPILib_auxiliary.h"


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
	double del_t =0.0;
	void get_delta_time();

	double accel_x_raw;
	double accel_y_raw;
	double gyro_raw;

	double gyro_raw_cos;
	double gyro_raw_sin;

	double jerk_x;
	double jerk_y;
	double accel_x_old;
	double accel_y_old;
	double accel_x_accum;
	double accel_y_accum;


	double x_final;
	double y_final;

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

	IMU();
	void Localization(BuiltInAccelerometer *accel,ADXRS450_Gyro *gyro);
};

#endif /* SRC_IMU_H_ */
