/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <WPIlib.h>
#include <SerialPort.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "rplidar41.h"
#include "RPLidar.h"


class Robot : public frc::IterativeRobot {
public:
	Joystick * joy;

//	rplidar_41 * drv;

	RPLidar * drv;

	double previous_angle = 0.0;




	bool toggle = 0;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	//	drv = new rplidar_41(115200, SerialPort::Port::kUSB1);
		joy = new Joystick(0);
		drv = new RPLidar();
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


		drv->end();


	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit()
	{
		drv->begin(SerialPort::Port::kUSB1);
		drv->startScan(true);
		drv->lidar_thread();
	//	drv->start();
		//drv->send_command(drv->command::RPLIDAR_CMD_SCAN,NULL,0);
	}

	void TeleopPeriodic()
	{
		/*
		if(joy->GetRawButton(1))
		{
		if(!toggle)
		drv->start();
		toggle = 1;
		}
		else
		toggle = 0;

		drv->get_measurement();
		printf("ANGLE %f: DISTANCE %f\n",drv->current.angle, drv->current.distance);
		*/
		/*
		if (IS_OK(drv->waitPoint())) {
		float distance = drv->getCurrentPoint().distance; //distance value in mm unit
		float angle    = drv->getCurrentPoint().angle; //anglue value in degree
		bool  startBit = drv->getCurrentPoint().startBit; //whether this point is belong to a new scan
		printf("d: %f  a: %f  s:  %b  \n",distance,angle,startBit);
		}
		else
		{
		drv->stop();
		drv->startScan(true);
		}
		*/

		//if (IS_OK(drv->waitPoint(0.002))) {
		float distance = drv->getCurrentPoint().distance; //distance value in mm unit
		float angle    = drv->getCurrentPoint().angle; //anglue value in degree
		bool  startBit = drv->getCurrentPoint().startBit; //whether this point is belong to a new scan
		//
		//make it something else... perhaps


		SmartDashboard::PutNumber("distance at 3degree", drv->angle_stuff[4].quality);//	//printf("%f\n", drv->angle_stuff[2]);
		SmartDashboard::PutNumber("angle at 3degree", drv->angle_stuff[4].angle);
		//	byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
		//  printf("d: %f  a: %f  s:  %d  \n",distance,angle,startBit);
		//perform data processing here...
		SmartDashboard::PutNumber("distance", distance);
		SmartDashboard::PutNumber("angle", angle);
		SmartDashboard::PutBoolean("startBit", startBit);
		//	SmartDashboard::PutBoolean("Is measuring", true);
		// } else {
		//	  SmartDashboard::PutBoolean("Is measuring", false);
		// }

	}

	void TestPeriodic() {}

	void disabledInit()
	{
		drv->stop();
	}


private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
