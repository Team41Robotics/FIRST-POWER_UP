/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#define K_P 1.0
#define K_I 0.0001
#define K_D 0.0001
//An ode to Matt Guo
//Parker is better than others that have been here. debatable.

#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>
#include "Driving.h"
#include "Lidar/RPLidar.h"


#define DRIVE_MODE 0 //0 = drive station; 1 = controller one stick.


class Robot : public frc::IterativeRobot {
public:

	double err_prev = 0.0;
	double acc = 0.0;
	double max_acc = 120.0;




	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		drv = new Driving();
		joyLeft = new Joystick(2);
		joyRight = new Joystick(1);
		controller = new Joystick(0);
		rightEncoder = new TalonSRX(5);
		leftEncoder = new TalonSRX(11);
		//enc = new Encoder(0,1);


		clawLinear = new TalonSRX(9);					//claw linear x
		arm0 = new TalonSRX(3);						//intake
		arm1 = new TalonSRX(10);					//intake
		armAct = new TalonSRX(4);					//rotate claw up

		lift = new TalonSRX(8);						//lift to move vertically


	//	test1 = new TalonSRX(6);
	//	test0 = new TalonSRX(7);


		arm_lim_out = new DigitalInput(2);
		arm_lim_in = new DigitalInput(3);
		topLift = new DigitalInput(0);
		bottomLift = new DigitalInput(1);

		//limit switches are plugged into Signal and Ground. They are 1 when open and 0 when closed.


	}

	TalonSRX * test1;
	TalonSRX * test0;

	DigitalInput * arm_lim_out;
	DigitalInput * arm_lim_in;

	DigitalInput * topLift;
	DigitalInput * bottomLift;


	Encoder * enc;

	TalonSRX * clawLinear;

	TalonSRX * rightEncoder;
	TalonSRX * leftEncoder;

	TalonSRX * arm0;
	TalonSRX * arm1;
	TalonSRX * armAct;






	TalonSRX * lift;

	Driving * drv;
	Joystick *joyLeft;
	Joystick *joyRight;
	Joystick *controller;
	double cmxn1;
	double cmxn2;
	double linearSpeed;

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}


		//=========		For grabbing a cube with camera		=======================
		/****************
		cmxn1 = SmartDashboard::GetNumber("cmxn1",0);
		double err = SmartDashboard::GetNumber("cmxn2",0);

		double out = K_P*err + K_I * acc + K_D * ( err - err_prev );
		//because html is garbage
		//use these.    -_-
		drv->move(-out,out);   //may want pid. this is just p.
		err_prev = err;
		acc += err;
		if(acc > max_acc)
			acc = max_acc;
		if(acc < -max_acc)
			acc = - max_acc;
		*********************/





	}

	void TeleopInit()
	{
		rightEncoder->GetSensorCollection().SetQuadraturePosition(0,500);
		leftEncoder->GetSensorCollection().SetQuadraturePosition(0,500);
	}

	void TeleopPeriodic()
	{


		if(DRIVE_MODE == 0)
			drv->tankDrive(joyLeft,joyRight);
		else if(DRIVE_MODE == 1)
			drv->ControllerMove(controller);

	//	test0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, controller->GetRawAxis(1));
	//	test1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, controller->GetRawAxis(5));





		//drv->tankDrive(controller->GetRawAxis(0));

//		double dinodan = enc->Get() * 2.8125;
//		SmartDashboard::PutNumber("enc",dinodan);
		//possibly add a bias to keep the arm up. something like +0.2
		clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -controller->GetRawAxis(0)*0.5);

		SmartDashboard::PutNumber("right encoder",rightEncoder->GetSensorCollection().GetQuadraturePosition()*(360.0/1024.0));
		SmartDashboard::PutNumber("left encoder",leftEncoder->GetSensorCollection().GetQuadraturePosition()*(360.0/1024.0));
		//SmartDashboard::PutNumber("that number",controller->GetRawAxis(5));

		/*if(joyRight->GetRawButton(1))
			armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1.0);
		if (joyLeft->GetRawButton(1))
			armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1.0);
		else
			armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);*/
		int pov = controller->GetPOV(0);
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

		if(controller->GetRawButton(1))
		{
			arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  controller->GetRawAxis(3));
			arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -controller->GetRawAxis(3));
		}
		else
		{
			arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -controller->GetRawAxis(3));
			arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  controller->GetRawAxis(3));
		}



		double lift_speed = -controller->GetRawAxis(5);
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



		//making the lifty thing do stuff with a slidy boy.
		// soodo cod:
		//#define LIFT_LOW 0.0
		//#define LIFT_HIGH xxxxxxx.xxxxxx			//this will be obtained experimentally. This is the encoder value of the highest position zeroed at the bottom.
			//make it move to a position based on Drive station sliders. assume there is a joystick named driver_station. the slider is Axis 0 [-1,1]



SmartDashboard::PutBoolean("that limit",topLift->Get());
printf("limit %d\n",topLift->Get());

	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
