/*
 * tankdrivetest.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#include <Driving.h>

Driving::Driving() {
	leftTalon0 = new TalonSRX(6);
	leftTalon1 = new TalonSRX(7);
	rightTalon0 = new TalonSRX(0);
	rightTalon1 = new TalonSRX(1);

	turbo = false;
	turbo_toggle = false;
	//leftTalon0 = new CANTalon(0);//these IDs have to be changed occasionally using the roborio-41-frc.local thing
	//rightTalon0 = new CANTalon(2);
	//leftTalon1 = new CANTalon(3);
	//rightTalon1 = new CANTalon(1);
}

void Driving::move(double leftspeed, double rightspeed)
{
	leftTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftspeed);
	rightTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightspeed);
	leftTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftspeed);
	rightTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightspeed);
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

void Driving::resetPosition()
{
	pos.reset();
}

double wrap(double a, double bounds)
{
	if(a < 0)
		return bounds + a;
	if(a >= bounds)
		return a - bounds;
	return a;
}

void Driving::updatePosition(TalonSRX * left, TalonSRX * right)
{
	left_n = left->GetSensorCollection().GetQuadraturePosition()*(PI/180.0)*WHEEL_RADIUS;		//say what it do
	right_n = right->GetSensorCollection().GetQuadraturePosition()*(PI/180.0)*WHEEL_RADIUS;
	left_d = left_n - left_o;
	right_d = right_n - right_o;



	if (fabs(left_d - right_d) < 1.0e-6) {
		pos.x += left_d * cos(pos.theta);
		pos.y += right_d * sin(pos.theta);
	    //pos.theta = pos.theta;
	} else {
	    float R = AXLE_LENGHT * (left_d + right_d) / (2 * (right_d - left_d));
	    float wd = (right_d - left_d) / AXLE_LENGHT;

	    pos.x += R * sin(wd + pos.theta) - R * sin(pos.theta);
	    pos.y -= R * cos(wd + pos.theta) + R * cos(pos.theta);
	    pos.theta = wrap(pos.theta + wd, 2 * PI); // forces it to be [0,2PI)
	}

	///23.5 5.75
}

void Driving::tankDrive(Joystick *joyLeft,Joystick *joyRight) {

	if(joyLeft->GetRawButtonPressed(1))		//toggles between turbo and normal speed modes
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


	//throttle = -1.0*clamp(joy1->GetRawAxis(1),-1.0,0.0)*(max_throt - min_throt) + min_throt;




	move(joyLeft->GetRawAxis(1), joyRight->GetRawAxis(1));

	//move(joyRight->GetRawAxis(1) - joyRight->GetRawAxis(0) , joyRight->GetRawAxis(1) + joyRight->GetRawAxis(0));


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

void Driving::ControllerMove(Joystick * controller)
{
	move(controller->GetRawAxis(1)*0.6 - controller->GetRawAxis(0)*0.6, controller->GetRawAxis(1)*0.6 + controller->GetRawAxis(0)*0.6);
}

void Driving::setEncoders()
{
	right_encoder = rightTalon0->GetSelectedSensorPosition(0);
	left_encoder = leftTalon0->GetSelectedSensorPosition(0);
	leftTalon0->GetSensorCollection().GetQuadraturePosition(); // this is also a way to access the encoder.
}


