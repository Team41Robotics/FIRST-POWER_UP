#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <CANTalon.h>
#include <Potentiometer.h>
//#include "LidarLite.h"

#define minAngle 0
#define maxAngle 90


class Robot: public frc::IterativeRobot {
public:
	double baseTarget, elbowTarget, clawTarget;
	CANTalon *baseMotor, *elbowMotor, *clawMotor;
	Potentiometer *basePot, *elbowPot, *clawPot;
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
		//Sample potentiometers - target angles
		baseTarget = 30;
		elbowTarget = 60;
		clawTarget = 50;
		//CANTalons
		baseMotor = new CANTalon(0);
		elbowMotor = new CANTalon(1);
		clawMotor = new CANTalon(2);
		//Potentiometers
		basePot = new Potentiometer()
	}

	void TeleopPeriodic() {
		//Get the angles from the gyros
		double baseAngle = baseGyro->GetAngle() % 360;
		double elbowAngle = elbowGyro->GetAngle() % 360;
		double clawAngle = clawGyro->GetAngle() % 360;
		//Acceptable error (in degrees)
		double error = 0.5;
		//Move the motors to reach those angles
		if(baseAngle < basePot - error) baseMotor->Set(1.0);
		if(baseAngle > basePot + error) baseMotor->Set(-1.0);
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
