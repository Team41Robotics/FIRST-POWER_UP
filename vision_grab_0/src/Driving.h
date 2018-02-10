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

#define PI 3.1415926
#define WHEEL_RADIUS 5.75/2.0
#define AXLE_LENGHT 23.5


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

    double throttle;
    double min_throt = 0.5;
    double max_throt = 0.9;
    bool turbo;
    bool turbo_toggle;

    double left_d, right_d, left_o, right_o, left_n, right_n;

public:
    Driving();
    void tankDrive(Joystick*joyLeft,Joystick*joyRight);
    void setEncoders();

    void resetPosition();
    void updatePosition(TalonSRX*left,TalonSRX*right);

    double right_encoder;
	double left_encoder;
	void move(double leftSpeed, double rightSpeed);

	position pos;
};


#endif
