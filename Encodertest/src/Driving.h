/*
 * tankdrivetest.h
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_
#include <WPILib.h>
#include "WPILib_auxiliary.h"

#include "ctre/Phoenix.h"

class Driving
{
private:
	TalonSRX * leftTalon0;
	TalonSRX * rightTalon0;
    TalonSRX * leftTalon1;
    TalonSRX * rightTalon1;

    TalonSRX * rt_encoder_talon;
    TalonSRX * lf_encoder_talon;

    void move(double leftSpeed, double rightSpeed);

    double throttle;
    double min_throt = 0.5;
    double max_throt = 0.9;
    bool turbo;
    bool turbo_toggle;

public:
    Driving();
    void tankDrive(Joystick*,Joystick*);
    void setEncoders();
    double right_encoder;
	double left_encoder;
};
#endif
