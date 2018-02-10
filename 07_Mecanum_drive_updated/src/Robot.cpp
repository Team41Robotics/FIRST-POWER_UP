#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <AnalogPotentiometer.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Driving.h>
#include <IMU.h>
#include <WPILib_auxiliary.h>
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <AHRS.h>

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Joystick *control_0;
	Joystick *control_1;

	AnalogInput *pot;
	CANTalon *shooterM1;
	CANTalon *shooterM2;
	FILE *shooterData;
	CANTalon *barrel;
	CANTalon *shooterIntake;
	CANTalon *intake;

//	ADXL345_I2C *accel_1;
	//SPI *test;
//	ADXRS450_Gyro *test;
//	NetworkTable *table;
	AHRS *nav;

	Timer *timer;

	bool intakeShooterSet;
	double getPotAngle()
	{
		double read = pot->GetVoltage();
		return (read/4.813)*300.0;
	}
/*
	double getPotAngle()
	{
		double read = pot->GetVoltage()-4.11;
		double retrun = (read/0.7079)*300.0;//check scale again (TEST AGAIN)   4.813
	}
*/

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motion_control = new Driving();
		control_0 = new Joystick(0);
		control_0 = new Joystick(1);

		//2 accelerometers because they both suck so maybe their sucking can interfere into no sucking?
/*		accel_0 = new BuiltInAccelerometer();
		accel_1 = new ADXL345_I2C(I2C::Port::kOnboard, Accelerometer::Range::kRange_4G);

		test = new ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);//make sure gyro has a jumper between desired port and channel
		test->Calibrate();
		test->Reset();*/
		//table->GetTable("localhost");

		nav = new AHRS(I2C::Port::kOnboard);

		//pot = new AnalogInput(2);
		intakeShooterSet = false;
		timer = new Timer();
		timer->Reset();
	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		shooterM1 = new CANTalon(4);
		shooterM2 = new CANTalon(8);

		barrel = new CANTalon(5);
		shooterIntake = new CANTalon(0);
		shooterIntake = new CANTalon(0);

		intake = new CANTalon(10);

		shooterData = fopen("shooterData.txt", "w");
	}

	void TeleopPeriodic() {

		if(control_1->GetRawButton(1)){
			shooterM1->Set(-((-control_1->GetRawAxis(3)+1.0)/2.0));
			shooterM2->Set(((-control_1->GetRawAxis(3)+1.0)/2.0));
			if(!intakeShooterSet){
				timer->Start();
				intakeShooterSet = true;
			}
			if(timer->Get()>0.5)
				shooterIntake->Set(0.25);
		}
		else{
			timer->Reset();
		}
		barrel->Set(-control_1->GetRawAxis(1));

		motion_control->Manual_driving(control_0);

		if(control_0->GetRawButton(1)){
			intake->Set((-control_0->GetRawAxis(3)+1.0)/2.0);
		}

		if (control_1->GetRawButton(4)){
			fprintf(shooterData, "%d\t%d\n", shooterM1->Get(), getPotAngle());
		}

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
