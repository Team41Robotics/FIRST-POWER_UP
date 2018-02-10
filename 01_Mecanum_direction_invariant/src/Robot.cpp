#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Driving.h>
#include <IMU.h>
#include <WPILib_auxiliary.h>
#include <Shooter.h>
#include <RobotDrive.h>

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Shooter *shooter;
	Joystick *control_0;
	BuiltInAccelerometer *accel_0;
	ADXL345_I2C *accel_1;
	//SPI *test;
	ADXRS450_Gyro *test;
//	std::shared_ptr<NetworkTable> table;
//	NetworkTable table;

	Timer *timer;
	double time = 0.0;

	Talon *schmidty;
	const float desired_angle = 10.0;

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motion_control = new Driving();
		//shooter = new Shooter();
		control_0 = new Joystick(0);
		//2 accelerometers because they both suck so maybe their sucking can interfere into no sucking?
		accel_0 = new BuiltInAccelerometer();
		accel_1 = new ADXL345_I2C(I2C::Port::kOnboard, Accelerometer::Range::kRange_4G);
		timer = new Timer();
		test = new ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);//make sure gyro has a jumper between desired port and channel
		test->Calibrate();
		test->Reset();
//		NetworkTable::SetIPAddress("10.0.41.62");
//		NetworkTable::SetTeam(41);
//		NetworkTable::SetClientMode();
//		table->GetTable("localhost");
		schmidty = new Talon(2);
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
	}
	void AutonomousPeriodic() {
		time = timer->Get()*1000;
		if(time<=15)
		{
			if (autoSelected == autoNameCustom) {
						//hier

						//timer
					if(time<=2)
					{
						motion_control->Auto_Move(0.25,0.25,0.25,0.25);
					}

					//distance

					//other
					} else {
						// Default Auto goes here
					}
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		//should be easy enough to do. Just setting it. The freshmen were tasked with this. They should have that stuff. I’ll add when we have the shooter.

		//oh, gee, what else do they want?



		//shooter->aim_shooter(shooter_encoder,shooter_pot);


		//motion_control->Auto_Move(control_0,test);

		SmartDashboard::PutNumber("fr",test->GetAngle());
		//motion_control->Auto_aim((test->GetAngle() - desired_angle)/desired_angle);

		//motion_control -> MecanumDrive_Cartesian()

		motion_control->Manual_driving (control_0, test);
		//motion_control->MecanumDrive_Cartesian(control_0->GetX(), m_driveStick->GetY(), m_driveStick->GetTwist());
		//motion_control -> MecanumDrive_Cartesian(control_0<-GetX(), control_0<-GetY(), control_0 <- GetTwist())
		//SmartDashboard::PutNumber("as",gyro_0->GetAngle());
	//	SmartDashboard::PutNumber("gf",test->GetAccumulatorValue());
//		table->PutNumber("X",3.0);
		//		printf("got %f",table->GetNumber("fr"));
//		schmidty->Set(control_0->GetRawAxis(3));
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
