/*
 * Lift.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: RoboWarriors
 */

#include <Lift.h>

Lift::Lift() {
	lift = new TalonSRX(Driving::TALON::LIFT);		//lift
	clawLinear = new TalonSRX(9);					//claw linear x
	arm0 = new TalonSRX(3);							//intake
	arm1 = new TalonSRX(10);						//intake
	armAct = new TalonSRX(4);						//rotate claw up
	lift = new TalonSRX(8);							//lift to move vertically

	arm_lim_out = new DigitalInput(2);
	arm_lim_in = new DigitalInput(3);
	topLift = new DigitalInput(0);
	bottomLift = new DigitalInput(1);

	//limit switches are plugged into Signal and Ground. They are 1 when open and 0 when closed.
	liftEncoder = new Encoder(8,9);
	linearSpeed = 0;
}

Lift::~Lift() {};

bool Lift::reset()
{

}

void Lift::sliderController(Joystick * diverstation)
{


	//drv->tankDrive(controller->GetRawAxis(0));

//		double dinodan = enc->Get() * 2.8125;
//		SmartDashboard::PutNumber("enc",dinodan);
	//possibly add a bias to keep the arm up. something like +0.2
	clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -diverstation->GetRawAxis(0)*0.5);

//	SmartDashboard::PutNumber("right encoder",rightEncoder->GetSensorCollection().GetQuadraturePosition()*(360.0/1024.0));
//	SmartDashboard::PutNumber("left encoder",leftEncoder->GetSensorCollection().GetQuadraturePosition()*(360.0/1024.0));
	//SmartDashboard::PutNumber("that number",controller->GetRawAxis(5));

	/*if(joyRight->GetRawButton(1))
		armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1.0);
	if (joyLeft->GetRawButton(1))
		armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1.0);
	else
		armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);*/
	int pov = diverstation->GetPOV(0);
	linearSpeed = 0;
	double deploySpeed = 0;
	if (pov != -1)			//POV is pressed
	{

		if (pov <= 135 && pov >=45)							//open claw
			linearSpeed = 1;
		if (pov <= 315 && pov >=225)						//close claw
			linearSpeed = -1;
		if (pov == 0 || pov == 315 || pov == 45)			//stow claw
			deploySpeed = 1;
		if (pov == 180 || pov == 135 || pov == 225)			//deploy claw
			deploySpeed = -1;
	}

	if( (linearSpeed > 0 && arm_lim_out->Get()) || (linearSpeed < 0 && arm_lim_in->Get()) )
		clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, linearSpeed);
	else
		clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);

	armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, deploySpeed);

	if(diverstation->GetRawButton(1))
	{
		arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  diverstation->GetRawAxis(3));
		arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -diverstation->GetRawAxis(3));
	}
	else
	{
		arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -diverstation->GetRawAxis(3));
		arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  diverstation->GetRawAxis(3));
	}



	double lift_speed = -diverstation->GetRawAxis(5);
	if(!topLift->Get())

	{
		//is hitting.
		if(lift_speed > 0.0)
			lift_speed = 0.0;
	}
	if(!bottomLift->Get())
	{
		//is hitting.
		if(lift_speed < 0.0)
			lift_speed = 0.0;
	}
	lift->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  lift_speed);



	//making the lifty thing do stuff with a slidy boi.
	//										edit: u/prateek boy->boi
	// soodo cod:
	//#define LIFT_LOW 0.0
	//#define LIFT_HIGH xxxxxxx.xxxxxx			//this will be obtained experimentally. This is the encoder value of the highest position zeroed at the bottom.
		//make it move to a position based on Drive station sliders. assume there is a joystick named driver_station. the slider is Axis 0 [-1,1]

//	SmartDashboard::PutNumber("the lift is at....",liftEncoder->Get());

//	SmartDashboard::PutBoolean("that limit",topLift->Get());
//	printf("limit %d\n",topLift->Get());

}
