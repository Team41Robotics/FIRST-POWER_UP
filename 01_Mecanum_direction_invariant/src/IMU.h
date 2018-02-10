#ifndef SRC_IMU_H_
#define SRC_IMU_H_
#include "WPILib.h"
#include "WPILib_auxiliary.h"

#define G 9.80665
#define RAD 3.14159265359 / 180.0

#define K_a0 0.5
#define K_a1 0.5

#define tau 0.45


class IMU
{
private:
	Timer *timer;


	double curr_t =0.0;
	double old_t =0.0;
	double del_t =0.0;
	void get_delta_time();

	double accel_x_raw;
	double accel_y_raw;
	double gyro_raw;

	double gyro_raw_cos;
	double gyro_raw_sin;


	double x_final;
	double y_final;

	double complementaryIMU(double accel_magnitude, double gyroAngle, int loop);
	void positionFilter(double ax, double ay, double ex, double ey);
public:
	double theta[2];
	double accel_x[2];
	double accel_y[2];
	double velocity_x[2];
	double velocity_y[2];
	double position_x[2];
	double position_y[2];
	IMU();
	void Localization(BuiltInAccelerometer *accel,ADXRS450_Gyro *gyro);
};

#endif /* SRC_IMU_H_ */
