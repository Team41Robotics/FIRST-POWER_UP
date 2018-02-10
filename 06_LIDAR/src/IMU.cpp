#include <IMU.h>

double x_bias=0;
double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
double Q_angle = 0.01;
double Q_gyro = 0.0003;
double R_angle = 0.01;

double K_0, K_1, S, y;

IMU::IMU()
{
	resetTimer();
	i = 0.0;
	double accel_x = 0;
	double accel_y = 0;
	double velocity_x = 0;
	double velocity_y = 0;
	double position_x = 0;
	double position_y = 0;
}

void IMU::resetTimer()
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
  //  x_final = a*(x_final + ex*del_t) + (1-a)*ax;
   // y_final = a*(y_final + ex*del_t) + (1-a)*ay;
}

double IMU::complementaryIMU(double accel_x, double gyroAngle, int loop){
	double dtC = loop/1000.0;
	double a = tau/(tau+del_t);
	double angle = a* (angle+ gyroAngle) + (1-a) * accel_x;
	return angle;
}

double IMU::kalmanIMU(double accelAngle, double gyroAngle){
	double angle = del_t * (gyroAngle - x_bias);
	P_00 += - del_t * (P_10 + P_01) + Q_angle * del_t;
	P_01 += - del_t * P_11;
	P_10 += - del_t * P_11;
	P_11 += Q_gyro * del_t;

	y = accelAngle - angle;
	S = P_00 + R_angle;

	K_0 = P_00 / S;
	K_1 = P_10 / S;

	angle += K_0 * y;
	x_bias += K_1 * y;



	angle += K_0 * y;
	x_bias += K_1 *y;
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;

	return angle;
}

double IMU::modulo(double i, double j){
	if(i < j)
		return i;
	i -= j;
	modulo(i,j);
}

void IMU::Localization (AHRS *nav)
{
	get_delta_time();
	//position_x = nav->GetDisplacementX()*39.37;
	//position_y = nav->GetDisplacementY()*39.37;
	//position_r = nav->GetYaw();
	theta = nav->GetAngle();
	accel_x = nav->GetRawAccelX();
	accel_y = nav->GetRawAccelY();
	double accel_mag = sqrt(accel_x * accel_x + accel_y * accel_y);
	velocity_x += cos(theta) * del_t * accel_mag;
	velocity_y += sin(theta) * del_t * accel_mag;
	position_x += velocity_x * del_t;
	position_y += velocity_y * del_t;
	SmartDashboard::PutNumber("x pos", velocity_x);
	SmartDashboard::PutNumber("y pos", velocity_y);
	SmartDashboard::PutNumber("x accel", accel_x);
	SmartDashboard::PutNumber("y accel", accel_x);
	SmartDashboard::PutNumber("x pos", position_x);
	SmartDashboard::PutNumber("y pos", position_x);
}

