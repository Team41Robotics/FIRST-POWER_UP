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
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Joystick *control_0;

	BuiltInAccelerometer *accel_0;
	ADXL345_I2C *accel_1;
	//SPI *test;
	ADXRS450_Gyro *test;

	float accumulated;
	const float K_P = 1.0;
	const float K_I = 0.0;
	const float K_D = 0.0;
	float out,prevError;
	const float maxAccumulated = 10.0;

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motion_control = new Driving();
		control_0 = new Joystick(0);
		//2 accelerometers because they both suck so maybe their sucking can interfere into no sucking?
		accel_0 = new BuiltInAccelerometer();
		accel_1 = new ADXL345_I2C(I2C::Port::kOnboard, Accelerometer::Range::kRange_4G);

		test = new ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);//make sure gyro has a jumper between desired port and channel
		test->Calibrate();
		test->Reset();

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

		SmartDashboard::PutNumber("fr",test->GetAngle());
		float desiredAngle = 20;
		float error = (test->GetAngle()-desiredAngle)/desiredAngle;
		error*=-1;

		accumulated += error;

		if(fabs(accumulated) > maxAccumulated)
			accumulated = maxAccumulated*(accumulated > 0 ? 1 : -1);

		out = K_P*error + K_I*accumulated + K_D*(-prevError + error);
		printf("Error%f\n Out:%f", error, out);
		prevError = error;

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
