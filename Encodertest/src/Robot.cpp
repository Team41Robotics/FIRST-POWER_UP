#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "WPILib_auxiliary.h"
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <Driving.h>
#include <Pneumatics.h>
#include <AHRS.h>
#include <Encoder.h>

class Robot: public frc::IterativeRobot {
public:
//	Pneumatics *pneu;
	Joystick * joyLeft;
	Joystick * joyRight;
	Driving *dr;
	Encoder *enc;
//	AHRS *nav;
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		joyLeft = new Joystick(1);
		joyRight = new Joystick(2);
		dr = new Driving();
		enc = new Encoder(0,1);
//\\
		'pneu = new Pneumatics();
	//	nav = new AHRS(SerialPort::Port::kUSB1);
	//	nav->Reset();
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

	}

	void TeleopPeriodic() {
		dr->setEncoders();//updates the encoder values.
		dr->tankDrive(joyLeft,joyRight);
	//	pneu->runFan();
	//	pneu->catapult(joyLeft);
		//printf("jlkdjkj:%f\n",joy->GetRawAxis(1));

		if(joyLeft->GetRawButton(3))
		{
		//	nav->is_moving = false;
		}

		SmartDashboard::PutNumber("the number",enc->Get());

/*
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


		frc::SmartDashboard::PutNumber("X:", nav->GetDisplacementX());
		frc::SmartDashboard::PutNumber("Y:", nav->GetDisplacementY());
		frc::SmartDashboard::PutNumber("Z:", nav->GetDisplacementZ());


*/
		if(joyLeft->GetRawButton(2))
		{
	//		nav->ResetDisplacement();
		//	nav->Reset();
		}

		joyLeft->GetButtonCount();


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
