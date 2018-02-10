/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <WPILib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <AHRS.h>

class Robot : public frc::IterativeRobot {
public:

	TalonSRX * tal_0;
	TalonSRX * tal_1;
	TalonSRX * tal_2;
	TalonSRX * tal_3;
	Joystick *joy_stick_0;

	AHRS *nav;
//leftTalon0 = new CANTalon(0);//these IDs have to be changed occasionally using the roborio-41-frc.local thing
	//rightTalon0 = new CANTalon(2);
	////leftTalon1 = new CANTalon(3);
	//rightTalon1 = new CANTalon(1);



	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		tal_0 = new TalonSRX(0);
		tal_1 = new TalonSRX(3);
		tal_2 = new TalonSRX(2);
		tal_3 = new TalonSRX(1);
		joy_stick_0 = new Joystick(0);
		nav = new AHRS(SerialPort::Port::kUSB1);
		nav->Reset();
	}

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
	}

	void TeleopInit() {}

	void TeleopPeriodic()
	{
		tal_0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, joy_stick_0->GetRawAxis(5));
		tal_1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, joy_stick_0->GetRawAxis(5));
		tal_2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, joy_stick_0->GetRawAxis(1));
		tal_3->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, joy_stick_0->GetRawAxis(1));
		printf("kl%f\n",joy_stick_0->GetRawAxis(5));
		printf("boy %f\n",nav->GetDisplacementX());
		frc::SmartDashboard::PutNumber("Pitch", nav->GetPitch());
		frc::SmartDashboard::PutNumber("Roll", nav->GetRoll());
		frc::SmartDashboard::PutNumber("Yaw", nav->GetYaw());
		frc::SmartDashboard::PutNumber("Compass", nav->GetCompassHeading());
		frc::SmartDashboard::PutBoolean("Calibrating?", nav->IsCalibrating());
		frc::SmartDashboard::PutBoolean("Connected?", nav->IsConnected());
		frc::SmartDashboard::PutBoolean("Moving?", nav->IsMoving());
		frc::SmartDashboard::PutBoolean("Rotating?", nav->IsRotating());
		frc::SmartDashboard::PutBoolean("Move does it?", nav->IsMoving());
		frc::SmartDashboard::PutNumber("Velocity X", nav->GetVelocityX());
		frc::SmartDashboard::PutNumber("Velocity Y", nav->GetVelocityY());
	}

	void TestPeriodic()
	{
		tal_boy_0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, jons_stick->GetRawAxis(5));
		tal_boy_1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, jons_stick->GetRawAxis(5));
		printf("kl%f\n",jons_stick->GetRawAxis(5));
	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
