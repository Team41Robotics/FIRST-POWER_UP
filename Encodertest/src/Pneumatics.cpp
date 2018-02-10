/*
 * Pneumatics.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#include <Pneumatics.h>

Pneumatics::Pneumatics() {
	rollerSolenoid = new DoubleSolenoid(2,3);
	shooterSolenoid = new DoubleSolenoid(0,1);
	fan = new Solenoid(7);
}
void Pneumatics::runCatapult(){
	shooterSolenoid->Set(DoubleSolenoid::Value::kReverse);
}
void Pneumatics::catapultDown(){
	shooterSolenoid->Set(DoubleSolenoid::Value::kForward);
}


//x is 3 and b is 2
void Pneumatics::catapult(Joystick *joy){
	if(joy->GetRawButtonPressed(3))
		runCatapult();
	if(joy->GetRawButtonPressed(2))
		catapultDown();
}

void Pneumatics::runFan(){
	fan->Set(true);
}
/*void Pneumatics::runS1(Joystick *joystick)
{
	//joystick->GetRawButton(1);
	if(joystick->GetRawButton(1))
	{
		rollerSolenoid->Set(DoubleSolenoid::Value::kForward);
	}
	else
	{
		rollerSolenoid->Set(DoubleSolenoid::Value::kReverse);
	}
}*/

Pneumatics::~Pneumatics()
{
	// TODO Auto-generated destructor stub
	//				destructor? nah.
}

