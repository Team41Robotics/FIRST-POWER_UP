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
#include "rplidar.h"
#include <rplidar41.h>
#include <SerialPort.h>

//RPlidarDriver* drv;

class Robot : public frc::IterativeRobot {
public:
	//RPlidarDriver *drv;
	//const char * opt_com_path = NULL;
	//_u32 opt_com_baudrate = 115200;
	//u_result op_result, op_starting;

	rplidar_41 * drv;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	//	drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
	//	opt_com_path = "COM0";
	//	//drv = drv->CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
		//RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
		//op_result = drv->connect(opt_com_path, opt_com_baudrate);
	///	op_starting = drv->startMotor();
///		drv->startScan();
		drv = new rplidar_41(115200,SerialPort::Port::kUSB1);
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

	void TeleopInit() {
//		drv->stop();
//		drv->stopMotor();
//		RPlidarDriver::DisposeDriver(drv);
		drv->stop();
	}

	void TeleopPeriodic() {}

	void TestPeriodic() {}

private:
//	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
