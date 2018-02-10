/*
 * PID.h
 *
 *  Created on: Feb 2, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include <WPILib.h>
#include "Driving.h"

#define Ki 0
#define Kp 0
#define Kd 0
#define PI 3.1415962
#define KiT 0
#define KpT 0
#define KdT 0
#define radius_wheel 0
#define radius_base 0
#define maxAcc 0
#define encRes 360

//#pragma once

class PID {
private:
	double err; //error
	double prvErr; //previous error
	double acc; //errors accumulated
	double current;
	double goal, angle;
	double out;
	Driving *drive;
	TalonSRX *enc_left, *enc_right;
	//for driving
public:
    PID(Driving* driving, TalonSRX *left_enc, TalonSRX *right_enc);
	void pid();
	void pidReset();
	bool drive_distance();
	bool turn_angle();
	void reset();
	void set_goal(double dist);
	void set_angle(double ang);
	int distOrAngle;
};

#endif /* SRC_PID_H_ */
