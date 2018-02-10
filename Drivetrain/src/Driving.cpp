/*
 * driving.cpp
 *
 *  Created on: Feb 2, 2018
 *      Author: RoboWarriors
 */

#include "Driving.h"

Driving::Driving() {
	//These parameters are subject to change
	leftBack = new TalonSRX(0);
	leftFront = new TalonSRX(1);
	rightBack = new TalonSRX(6);
	rightFront = new TalonSRX(7);
	//	leftTalon0 = new TalonSRX(0);
//	rightTalon0 = new TalonSRX(6);
//	leftTalon1 = new TalonSRX(1);
//	rightTalon1 = new TalonSRX(7);

	enc = new Encoder(1,2);

}

void Driving::move(double leftSpeed, double rightSpeed) {
	leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftSpeed);
	leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftSpeed);
	rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightSpeed);
	rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightSpeed);
}


void Driving::stop() {
	move(0.0, 0.0);
}
