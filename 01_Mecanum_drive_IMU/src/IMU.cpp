#include <IMU.h>

IMU::IMU ()
{
	timer = new Timer();
	timer->Reset();
	timer->Start();
}

void IMU::get_delta_time ()
{
	curr_t = timer->Get();    //unsure what units. need it in seconds.
	del_t = curr_t - old_t;
	old_t = curr_t;
}

void IMU::positionFilter(double ax, double ay, double ex, double ey){
    double a = tau/(tau+del_t);
    x_final = a*(x_final + ex*del_t) + (1-a)*ax;
    y_final = a*(y_final + ex*del_t) + (1-a)*ay;
}

double IMU::complementaryIMU(double accel_magnitude, double gyroAngle, int loop){
	double dtC = loop/1000;
	double a = tau/(tau+del_t);
	double angle = a* (angle+ gyroAngle) + (1-a) * accel_magnitude;
	return angle;
}

void IMU::Localization (BuiltInAccelerometer *accel,ADXRS450_Gyro *gyro)
{
	get_delta_time();

	accel_x_raw = accel->GetX();//+ error correction
	accel_y_raw = accel->GetY();//+ error correction
	gyro_raw = gyro->GetAngle()*RAD;
	gyro_raw_cos = cos(gyro_raw);
	gyro_raw_sin = sin(gyro_raw);

	positionFilter(accel_x_raw,accel_y_raw,gyro_raw_cos,gyro_raw_sin);


	accel_x = ( accel->GetX() * K_a0 + accel->GetX() * K_a1 ) * G;
	// K_a are accelerometer constants. Ask Neelay. He should know a good value. Also see him regarding iterative filtering to reduce error.
	accel_y = ( accel->GetY() * K_a0 + accel->GetY() * K_a1 ) * G;

	theta = gyro->GetAngle() * RAD;

	velocity_x += ( accel_x * cos(theta[0]) + accel_y * sin(theta[0]) ) * del_t;
	velocity_y += ( accel_y * cos(theta[0]) + accel_x * sin(theta[0]) ) * del_t;

	position_x += velocity_x * del_t;//very high in accError
	position_y += velocity_y * del_t;
}
