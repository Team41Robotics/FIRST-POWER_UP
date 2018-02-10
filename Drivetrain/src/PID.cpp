/*
 * PID.cpp
 *
 *  Created on: Feb 2, 2018
 *      Author: RoboWarriors
 */

#include <PID.h>

PID::PID(Driving* driving, TalonSRX *left_enc, TalonSRX *right_enc) {
    this->drive = driving;
    this->enc_left = left_enc;
    this->enc_right = right_enc;
    distOrAngle = 0;
    acc = 0;
    current = 0;
    angle = 0;
    out = 0;
    err = 0;
    prvErr = 0;
    goal = 0;
}

void PID::set_goal(double dist) {
    goal = dist;
    reset();
    distOrAngle = 1;
}

void PID::set_angle(double ang) {
    angle = ang;
    reset();
    distOrAngle = 2;
}

//Drives to the inputted distance
bool PID::drive_distance() {
	double dist_current = 2 * PI * radius_wheel / encRes * (enc_right->GetSelectedSensorPosition(0) + enc_left->GetSelectedSensorPosition(0)) / 2.0;

	acc = 0;

	if(dist_current>=goal)
	{
		drive->stop();
		return true;
	}
	err = dist_current - goal;
	acc += err;
	if(acc > maxAcc)
		acc = maxAcc;
	if(acc < -maxAcc)
		acc = -maxAcc;
	out = Kp*err+ Ki*acc + Kd*(err - prvErr);
	prvErr =  err;

	drive->move(out, out);
    return false;
}

//Turns to the inputted angle
bool PID::turn_angle() {
	float arc = (enc_right->GetSelectedSensorPosition(0) - enc_left->GetSelectedSensorPosition(0)) * PI * radius_wheel / 360;
	float angle_current = arc * 180 / PI * radius_base;

	if(fabs(angle_current) >= fabs(angle)) {
		drive->stop();
		return true;
	}
	err = angle - angle_current;
	acc += err;
	if(acc > maxAcc)
		acc = maxAcc;
	if(acc < -maxAcc)
		acc = -maxAcc;
	out = KpT*err+ KiT*acc + KdT*(err - prvErr);
	prvErr =  err;

	//now make the wheels move…
	//this move is assuming move is (left, right)
	drive->move(out, -out);
	return false;
}

//Resets TalonSRXs and PID
void PID::reset() {
	//(int sensorPos, int pidIdx, int timeoutMs)
	enc_left->SetSelectedSensorPosition(0, 0, 500);
	enc_right->SetSelectedSensorPosition(0, 0, 500);
	pidReset();
	distOrAngle = 0;
}

//Resets PID
void PID::pidReset() {
	acc = 0;
	err= 0;
	prvErr= 0;
}
