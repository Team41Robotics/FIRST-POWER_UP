/*
 * tankdrivetest.h
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_
#include <WPILib.h>

#include "ctre/Phoenix.h"
#include "states.h"
#include <AHRS.h>
#include "Lidar/RPLidar.h"
#include "Lift.h"
#define DOMINQUE 0

#define PI 3.1415926
#define WHEEL_RADIUS (5.75/2.0)
#define WHEELBASE 22.5
#define K_P (1.0/60.0)	//this is the limit of when it will start slowing down. When it is within 1/N where N is in inches.
#define TOLERANCE 1.0

struct position
{
	position ()
	{
		x=0.0;
		y=0.0;
		theta=0.0;
	}
	void reset()
	{
		x=0.0;
		y=0.0;
		theta=0.0;
	}
	double x;
	double y;
	double theta;
};

class Driving
{
private:
	TalonSRX * leftTalon0;
	TalonSRX * rightTalon0;
    TalonSRX * leftTalon1;
    TalonSRX * rightTalon1;

    TalonSRX * rt_encoder_talon;
    TalonSRX * lf_encoder_talon;


    double throttle=0;
    double min_throt = 0.5;
    double max_throt = 0.9;
    bool turbo=0;
    bool turbo_toggle=0;

    double left_d=0, right_d=0, left_o=0, right_o=0, left_n=0, right_n=0;


	double k_p = 1.0/50;//50;
	double k_i = 0.004;//0.00175;
	double current;

	double k_iAngle = 0.0049;//0.0045;//0.008;//0.003;
	double k_pAngle = 0.5;//0.5;//0.7;//0.4;
	double k_dAngle = 0.0;//0.01;//0.7;//0.4;

	double current_angle = 0;
	double previous_angle;
	bool firstTime = true;
	void resetAuto();
	double k_pCube = 0.48;//0.4;
	double k_iCube = 0.002;//0.0018;
	double k_dCube = 0.000;
	double previous = 0.0;
public:
    Driving();
  //  void tankDrive(Joystick*joyLeft,Joystick*joyRight);
    void tankDrive(Joystick*joyLeft,Joystick*joyRight,Joystick *buttonBoard = NULL);
    void setEncoders();
	bool autoCube(Lift * lift);
    void resetPosition();
    void updatePosition();
    bool AutoForwards(double goal);
    bool AutoTurn(double angle, double tolerance = 2.0);
    bool LidarForwards(double goal, RPLidar * lidar);
    void resetEncoders();
    double right_encoder;
    double left_encoder;
	double integral_angle;
	double integral;
	double speed;
	double error_angle;
	double error;


    void drive(double leftSpeed, double rightSpeed);

	void ControllerMove(Joystick * controller);
	position pos;
    AHRS * wharhs;

	/*
	 * leftTalon0 = new TalonSRX(6);
	leftTalon1 = new TalonSRX(7);
	rightTalon0 = new TalonSRX(0);
	rightTalon1 = new TalonSRX(1);
	Arm0 = new TalonSRX(3);						//intake
	arm1 = new TalonSRX(10);					//intake
	armAct = new TalonSRX(4);					//rotate claw up

	lift = new TalonSRX(8);
	 */

	//THIS ONE IS DOMINQUE
#ifdef DOMINQUE
	enum TALON {
			RIGHT_FRONT = 12,
			RIGHT_BACK = 1,
			LEFT_FRONT = 6,
			LEFT_BACK = 7,
			LIFT = 8,
			CLAW_0 = 3,
			CLAW_1 = 2,
			CLAW_ROTATION = 4,
			CLAW_LINEAR = 9,
			RIGHT_ENC = -1,		//right and left encoder are on 0 & 6?
			LEFT_ENC = -1,
			RAMP_0 = 10
		};
#else
	//THIS ONE IS BUBBLES
	enum TALON {
		RIGHT_FRONT = 5,
		RIGHT_BACK = 1,
		LEFT_FRONT = 11,
		LEFT_BACK = 6,
		LIFT = 8,
		CLAW_0 = 2,
		CLAW_1 = 7,
		CLAW_ROTATION = 4,
		CLAW_LINEAR = 9,
		RIGHT_ENC = 5,
		LEFT_ENC = 11
	};
#endif
};


#endif
