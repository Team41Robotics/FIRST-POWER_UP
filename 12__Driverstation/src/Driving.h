#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_
#include "WPILib.h"
#include "WPILib_auxiliary.h"
#define DRIVING_ENABLE
#define K_aim_p 1
#define K_aim_i 0
#define K_aim_d 0
#define K_drive_p 1
#define K_drive_i 0
#define K_drive_d 0
#define K_turn_p 1
#define K_turn_i 0
#define K_turn_d 0
#define MAX_AIM_ACCUM 1
#define MAX_DRIVE_ACCUM 1
#define MAX_TURN_ACCUM 1
#define RAD 3.14159265359 / 180.0
#include <LidarLite.h>
#include <IMU.h>
#include <I2C.h>

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
	double drive_error;
	double drive_error_accum;
	double drive_error_old;
	double drive_out;
	double turn_error;
	double turn_error_accum;
	double turn_error_old;
	double turn_out;

	int firstDist = -1;
	//AHRS *nav;
	//IMU *imu;
	double max(double a, double b);
	double absD(double a);
	float throtTemp = 0.7;

public:
	AHRS *nav;
	IMU *imu;
	LidarLite *lidar;

	Driving();
	int LidarDist();
	void Auto_aim(double cmxn);
	void Move(double fr, double fl, double br, double bl);
	void Mecanum_drive(Joystick *control_joy);
	void Manual_driving(Joystick *control_joy);
	void Auto_Move ( double fr , double fl , double br , double bl );
	bool NOPID_Turn (float angle,float Speed);
	bool Pid_Move_upTo (double Dist);
	bool Pid_Move_downTo (double Dist);
	bool NOPID_Move (float Dist,float Speed,bool Foward = true);
	bool NOPID_Move_Dist (float Dist,float Speed);
	void Auto_Foward (double speed);
	bool Pid_turn_downTo(double angle);
	bool Pid_turn_upTo (double angle);
	void Auto_Stop();
};




#endif /* SRC_DRIVING_H_ */
