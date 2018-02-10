/*
 * tankdrivetest.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#include <Driving.h>

Driving::Driving() {
	leftTalon0 = new TalonSRX(0);
	rightTalon0 = new TalonSRX(6);
	leftTalon1 = new TalonSRX(1);
	rightTalon1 = new TalonSRX(7);

	turbo = false;
	turbo_toggle = false;
	//leftTalon0 = new CANTalon(0);//these IDs have to be changed occasionally using the roborio-41-frc.local thing
	//rightTalon0 = new CANTalon(2);
	//leftTalon1 = new CANTalon(3);
	//rightTalon1 = new CANTalon(1);
}

void Driving::move(double leftspeed, double rightspeed) {
	leftTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, leftspeed);
	rightTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -rightspeed);
	leftTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, leftspeed);
	rightTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -rightspeed);
}

double signum(double a)
{
	if(a < 0)
		return -1.0;
	return 1.0;
}

double clamp(double val, double low, double high)
{
	if(val > high)
		return high;
	if(val <low)
		return low;
	return val;
}

double magnitude (double a, double b)
{
	return sqrt(a*a+b*b);
}

void Driving::tankDrive(Joystick *joy1,Joystick *joy2) {

	if(joy1->GetRawButtonPressed(1))
	{
		if(!turbo_toggle)
		{
			turbo = !turbo;
		}
		turbo_toggle = 1;
	}
	else
	{
		turbo_toggle = 0;
	}


	throttle = -1.0*clamp(joy1->GetRawAxis(1),-1.0,0.0)*(max_throt - min_throt) + min_throt;


	//this should be one joystick tank drive.
	//move(joy2->GetRawAxis(1)*throttle - joy2->GetRawAxis(2)*throttle, joy2->GetRawAxis(1)*throttle + joy2->GetRawAxis(2)*throttle);
	move(joy1->GetRawAxis(1), joy2->GetRawAxis(1));
	//move(magnitude(joy2->GetRawAxis(0),joy2->GetRawAxis(1))*0.6 - joy2->GetRawAxis(2)*0.6, magnitude(joy2->GetRawAxis(0),joy2->GetRawAxis(1))*0.6 + joy2->GetRawAxis(2)*0.6);

	if(turbo)
	{
		//	move(joy1->GetRawAxis(1)*0.6, joy2->GetRawAxis(1)*0.6);
	}
	else
	{
		//	move(joy1->GetRawAxis(1)*0.6, joy2->GetRawAxis(1)*0.6);
		//move(joy1->GetRawAxis(1)/4, joy2->GetRawAxis(1)/4);
		//move(joy1->GetRawAxis(1)/4, joy1->GetRawAxis(5)/4);
	}

}

void Driving::setEncoders()
{
	right_encoder = rightTalon0->GetSelectedSensorPosition(0);
	left_encoder = leftTalon0->GetSelectedSensorPosition(0);
	leftTalon0->GetSensorCollection().GetQuadraturePosition(); // this is also a way to access the encoder.
}


