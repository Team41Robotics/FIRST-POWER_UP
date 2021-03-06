#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_
#include "WPILib.h"
#include "WPILib_auxiliary.h"
#define DRIVING_ENABLE
#define K_aim_p 1
#define K_aim_i 0
#define K_aim_d 0
#define MAX_AIM_ACCUM 1
#define RAD 3.14159265359 / 180.0
class Driving
{
private:




	CANTalon *M_fr;                  //declarations for the talons (motor controllers)
	CANTalon *M_fl;
	CANTalon *M_br;
	CANTalon *M_bl;



	double throt;
	double throt_min;
	double throt_rng;

	double con_x;
	double con_y;
	double con_t;

	double theta;

	double angle_factor_pp;
	double angle_factor_pm;
	double angle_factor_mp;
	double scl;

	int direction_angle;

	double aim_error;
	double aim_error_accum;
	double aim_error_old;
	double aim_out;
	ADXRS450_Gyro *gyro;

	double max(double a, double b);
	double absD(double a);

public:

	Driving();
	void Auto_aim(double cmxn);
	void Move(double fr, double fl, double br, double bl);
	void Mecanum_drive(Joystick *control_joy);
	void Manual_driving(Joystick *control_joy);
	void Auto_Move ( double fr , double fl , double br , double bl );
};




#endif /* SRC_DRIVING_H_ */
