#include <iostream>
#include <string>
#include <WPILib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <fstream>
#include "ctre/Phoenix.h"
#include <thread>

class Robot : public frc::IterativeRobot {
public:

	TalonSRX *frontRight;
	TalonSRX *frontLeft;
	TalonSRX *backRight;
	TalonSRX *backLeft;

	Joystick *joyLeft;
	Joystick *joyRight;




	bool is_recording = 0;

	bool on_toggle = 0;
	bool auto_done = 0;
	bool off_toggle = 0;

	std::string file_name = "/home/lvuser/auto1.txt";

	std::ifstream file_in;
	std::ofstream file_out;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		frontRight = new TalonSRX (2);
		frontLeft = new TalonSRX (0);
		backRight = new TalonSRX (1);
		backLeft = new TalonSRX (3);
		joyLeft = new Joystick (2);
		joyRight = new Joystick (1);
		/*
		 *
	leftTalon0 = new TalonSRX(0);
	rightTalon0 = new TalonSRX(2);
	leftTalon1 = new TalonSRX(3);
	rightTalon1 = new TalonSRX(1);
		 */
	}

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

		auto_done = 0;
		file_out.close();
		file_in.open(file_name);
	}





	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

		if(!auto_done)
		{
			if(!file_in.eof()) // checks if it is end of file
			{
				//move based on input
				std::string right;
				std::string left;
				file_in >> right >> left;

				//THIS IS FOR WHEN ENCODERS ARE PUT ON
				//std::string enc_left_str;
				//std::string enc_right_str;
				//file_in >> enc_right_str >> enc_left_str;
				//double enc_rt = enc_right_str.c_str();
				//double enc_lf = enc_left_str.c_str();
				//double enc_rt_dif = enc_rt - encoder_right->Get();
				//double enc_lf_dif = enc_lf - encoder_left->Get();
				//#define ENC_COEFFICENT = 0.0000  // to be put at the top...
				//right_spd = right_spd + ENC_COEFFICIENT * enc_rt_dif;
				//left_spd = left_spd + ENC_COEFFICIENT * enc_lf_dif;


				double right_spd = std::atof(right.c_str());
				double left_spd = std::atof(left.c_str());
				frontRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, right_spd);
				frontLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -left_spd);
				backRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, right_spd);
				backLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -left_spd);
				printf("right %f, left %f\n",right_spd, left_spd);
			}
			else
			{
				if(!auto_done)
				{
					file_in.close();
					auto_done = 1;
				}
				frontRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
				frontLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
				backRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
				backLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
			}
		}

	}

	void TeleopInit()
	{
		file_in.close();
	}

	void TeleopPeriodic()
	{
		double right_speed = joyRight->GetRawAxis(1);
		double left_speed = joyLeft->GetRawAxis(1);


		frontRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, right_speed*0.6);
		frontLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -left_speed*0.6);
		backRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, right_speed*0.6);
		backLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -left_speed*0.6);

		//THIS IS FOR WHEN ENCODERS ARE PUT ON
		//double rightEnc = encoder_right->Get();
		//double leftEnc = encoder_left->Get();


		if(is_recording)
		{
			file_out << right_speed*0.6 << " " << left_speed*0.6 << "\n";
			//THIS IS FOR WHEN ENCODERS ARE PUT ON
			//file_out << rightEnc << " " << leftEnc << "\n";
			printf("right %f, left %f\n",right_speed*0.6, left_speed*0.6);
		}



		if(joyRight->GetRawButton(1))
		{
			if(!on_toggle)
			{
				is_recording = 1;
				file_out.open(file_name);
				printf("opened\n");
			}
			on_toggle = 1;
		}
		else
		{
			on_toggle = 0;
		}

		if(joyLeft->GetRawButton(1))
		{
			if(!off_toggle)
			{
				is_recording = 0;
				file_out.close();
				printf("closed\n");
			}
			off_toggle = 1;
		}
		else
		{
			off_toggle = 0;
		}


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
