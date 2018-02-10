#include <IMU.h>

double x_bias=0;
double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
double Q_angle = 0.01;
double Q_gyro = 0.0003;
double R_angle = 0.01;

double K_0, K_1, S, y;
/*
IMU::IMU()
{
	timer = new Timer();
	timer->Reset();
	timer->Start();
	i = 0.0;
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

void IMU::Localization (BuiltInAccelerometer *accel,ADXRS450_Gyro *gyro)
{
	get_delta_time();

	accel_x_raw = floor((accel->GetX()*385.827)*1000.0)/1000.0;// +0.090;//-0.640038364691114+0.745298706953019+0.02940776758838014-0.025;//+ error correction
	accel_y_raw = floor((accel->GetY()*385.827)*1000.0)/1000.0;// +1.762;//+0.17289889102453315+1.5208357071897574+0.07700778237031058-0.06;//+ error correction

	jerk_x = (accel_x_raw - accel_x_old);
	jerk_y = (accel_y_raw - accel_y_old);
	accel_x_old = accel_x_raw;
	accel_y_old = accel_y_raw;

	accel_x_accum += jerk_x * del_t;
	accel_y_accum += jerk_y * del_t;

	SmartDashboard::PutNumber("Accel_x",);
	SmartDashboard::PutNumber("Accel_y",);

	//gyro_raw = gyro->GetAngle()*RAD;
	double gyro_raw1 = gyro->GetAngle();
	gyro_raw1 = modulo(gyro_raw1, 360.0);

	double filtered= complementaryIMU(accel_x_raw, gyro_raw1, 1);
	double kalman  = kalmanIMU(accel_x_raw, gyro_raw1);

	SmartDashboard::PutNumber("Filtered", filtered);
	SmartDashboard::PutNumber("Kalman", kalman);
	SmartDashboard::PutNumber("Gyro", gyro_raw1);

	gyro_raw_cos = cos(gyro_raw1*RAD);
	gyro_raw_sin = sin(gyro_raw1*RAD);

//	positionFilter(accel_x_raw,accel_y_raw,gyro_raw_cos,gyro_raw_sin);


	accel_x = ( accel->GetX() * K_a0 + accel->GetX() * K_a1 ) * G;
	// K_a are accelerometer constants. Ask Neelay. He should know a good value. Also see him regarding iterative filtering to reduce error.
	accel_y = ( accel->GetY() * K_a0 + accel->GetY() * K_a1 ) * G;

	theta = gyro->GetAngle() * RAD;

	//theta = gyro->
	//double test_velocity_x += accel_x * dt;
	//double test


	velocity_x += accel_x * del_t;//( accel_x * gyro_raw_cos + accel_y * gyro_raw_sin ) * del_t;
	velocity_y += accel_y * del_t;//( accel_y * gyro_raw_cos + accel_x * gyro_raw_sin ) * del_t;

	SmartDashboard::PutNumber("VEl_x",velocity_x);
	SmartDashboard::PutNumber("VEL_y",velocity_y);
	/*position_x += velocity_x * del_t;//very high in accError
	position_y += velocity_y * del_t;

	position_x += velocity_x * del_t;//((accel_x * (0.004))/2);//* gyro_raw_cos;
	position_y += velocity_y * del_t;//((accel_x * (0.004))/2);//* gyro_raw_sin;

	SmartDashboard::PutNumber("position X",position_x);
	SmartDashboard::PutNumber("position Y",position_y);
}
*/
