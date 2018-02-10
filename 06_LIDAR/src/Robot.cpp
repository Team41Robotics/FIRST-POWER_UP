#include <AHRS.h>
#include <WPILib.h>
#include <CANTalon.h>
#include <Counter.h>
#include <DigitalInput.h>
#include <Driving.h>
#include <interfaces/Potentiometer.h>
#include <I2C.h>
#include <IMU.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotBase.h>
#include <Shooter.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Timer.h>
#include <cstdio>
#include <iostream>
#include <string>
#include "LidarLite.h"

#define totalDimension 162
#define dim1 25 //we’ll need to change this

int  i=0;

//#include <Pot.h>

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Shooter *shooter;
	IMU *imu;
	Joystick *control_0;
	AHRS *nav;
	//AnalogInput * test2;
	//PWM *test;
	LidarLite *lidar;
	DigitalInput *test3;
	Counter *test4;
	CANTalon *tal;
	Timer *timer;
	double time = 0.0;
	Potentiometer *pot;

	/*double autonDriveSTR(double velocity);
	double autonDriveTurn(double velocity);
*/

	CANTalon *shooterM2;
	CANTalon *shooterM1;
	CANTalon *barrel;

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motion_control = new Driving();
		shooter = new Shooter();
		control_0 = new Joystick(0);
		timer = new Timer();
		timer->Start();
		nav = new AHRS(I2C::Port::kOnboard);
		nav->Reset();
		imu = new IMU();
	//	test = new PWM(5);
		tal = new CANTalon(10);
		//test2 = new AnalogInput(3);
		test3 = new DigitalInput(4);
		test4 = new Counter(3);
		//pot = new AnalogPotentiometer(0, 300, 0);

		shooterM1 = new CANTalon(4);
		shooterM2 = new CANTalon(8);
		lidar = new LidarLite(I2C::Port::kOnboard,0X62);
		barrel = new CANTalon(5);
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
			SmartDashboard::PutString("Auton","doop");
		} else {
			// Default Auto goes here
			SmartDashboard::PutString("Auton","preston");
		}
		timer->Reset();
		timer->Start();

	}
	void AutonomousPeriodic() {
		time = timer->Get();
		if (autoSelected == autoNameCustom) {
			//imu->Localization(nav);
			if(imu->position_y < 114)
			{
				motion_control->Auto_Move(-0.25,-0.25,-0.25,-0.25);
			}
			else if(imu->theta < 30)
			{
				motion_control->Auto_Move(+0.25,-0.25,+0.25,-0.25);
			}
			else if(imu->position_x < 12.5)
			{
				motion_control->Auto_Move(-0.50,-0.50,-0.50,-0.50);;
			}

		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
		lidar->reset();
	}

	void TeleopPeriodic() {

		lidar->reset();
		if(lidar->isMeasurementValid(false))
			SmartDashboard::PutNumber("lidar",lidar->getDistance()*0.394);

		motion_control->Manual_driving(control_0);
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
