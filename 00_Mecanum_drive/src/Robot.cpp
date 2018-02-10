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
#include "LidarLite.h"

//#define CAM_SERVO_ON

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Joystick *control_0;
	Joystick *control_1;

	CANTalon *shooterM1;
	CANTalon *shooterM2;
	CANTalon *barrel;
	CANTalon *shooterIntake;
	CANTalon *intake;
	CANTalon *climber;
	Servo *cam_servo;

	AnalogInput *pot;

	BuiltInAccelerometer *accel_0;
//	ADXL345_I2C *accel_1;
	//SPI *test;
//	ADXRS450_Gyro *test;
	NetworkTable *table;

//	Talon *shooterIntake_Aux_Left;
//	Talon *shooterIntake_Aux_Right;
	Talon *shooterIntake_Aux_Left;
	Talon *shooterIntake_Aux_Right;

	Timer *timer;

	Relay *light;

	LidarLite *lidar;

	double cam_servo_angle = 0.0;
	double cam_servo_angle_adjust = 0.0;
	double cam_servo_tolerance = 0.0;

//	CameraServer::GetInstance()->StartAutomaticCapture();
//
/*	double totalDimension = 162;
	double dim1  = 25; //we’ll need to change this
	int  auton_i=0;
	*/


	double getPotAngle()
	{
		double read = pot->GetVoltage()-4.11;
		double angle = (read/0.7079)*300.0;//check scale again (TEST AGAIN)   4.813
		SmartDashboard::PutNumber("Raw pot", read);
		SmartDashboard::PutNumber("Pot Angle:", angle);
		return angle;
	}


	bool intakeShooterSet;
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motion_control = new Driving();
		control_0 = new Joystick(0);
		control_1 = new Joystick(1);
		cam_servo = new Servo(2); /// pwm
		shooterIntake_Aux_Left = new Talon(0);// Servo(0);
		shooterIntake_Aux_Right = new Talon(1);// Servo(9);

		lidar = new LidarLite(I2C::Port::kOnboard,0X62);

		//2 accelerometers because they both suck so maybe their sucking can interfere into no sucking?
/*		accel_0 = new BuiltInAccelerometer();
		accel_1 = new ADXL345_I2C(I2C::Port::kOnboard, Accelerometer::Range::kRange_4G);

		test = new ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);//make sure gyro has a jumper between desired port and channel
		test->Calibrate();
		test->Reset();*/
		shooterM1 = new CANTalon(4);
		shooterM2 = new CANTalon(8);

		barrel = new CANTalon(5);
		shooterIntake = new CANTalon(1);

		intake = new CANTalon(10);

		climber = new CANTalon(2);

		table->GetTable("localhost");

		pot = new AnalogInput(0);

		intakeShooterSet = false;
		timer = new Timer();
		timer->Reset();

		light = new Relay(0);
	}

	void AutonomousInit() override {

	}
	void AutonomousPeriodic() {
/*
		int movement = (139.3*sqrt(3)-((totalDimension)-dim1))/sqrt(3);
		double velocity = 0.5;
		if (auton_i == 0)//move forward before turn
		{
		    if(imu->position_y>=movement)
		    {
		    	auton_i=1;
		        imu->resetpos();
		    }
		    Move(velocity,velocity,velocity,velocity);
		    }
		else if(auton_i ==1)//turn (60 deg)
		{
		if(imu->theta>= 60)
		{
			auton_i=2;
			imu->resetpos();
		}
		Move(velocity,-velocity,velocity,-velocity);		}
		else if (auton_i==2)//move to gear drop location
		{
		    //lidar stuff
			imu->resetpos();
		}
		else
		{
		    //get out of auton
		}
*/
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

		//servo:
		//get servo value somewhere; Possibly user input or compute. For this it will grab from smartdash

#ifdef CAM_SERVO_ON
		cam_servo_angle = 90 + cam_servo_angle_adjust; //so it returns the proper angle from our chosen reference frame
		cam_servo->SetAngle(cam_servo_angle);
		if(fabs(cam_servo->GetAngle()-cam_servo_angle) <= cam_servo_tolerance) {
			SmartDashboard::PutNumber("On Angle",true);
		} else {
			SmartDashboard::PutNumber("On Angle",false);
		}
#endif



/*		if(control_1->GetRawButton(1)){
			shooterM1->Set(-((-control_1->GetRawAxis(3)+1.0)/2.0));
			shooterM2->Set(((-control_1->GetRawAxis(3)+1.0)/2.0));
			printf("%f\n",((-control_1->GetRawAxis(3)+1.0)/2.0));
//.769531 -> 48 in.
//
			if(!intakeShooterSet){
				timer->Start();
				intakeShooterSet = true;
			}
			if(timer->Get()>0.5)
				shooterIntake->Set(0.8);//pretty much tested, any value higher is not gonna make a difference -N
	 	}*/
/*		if (control_1->GetRawButton(11)){
			shooterM1->Set(-0.742188);
			shooterM2->Set(0.742188);
			if(!intakeShooterSet){
				timer->Start();
				intakeShooterSet = true;
			}
			if(timer->Get()>0.5)
				shooterIntake->Set(0.8);//pretty much tested, any value higher is not gonna make a difference -N
		}*/
/*		else{
			timer->Reset();
			shooterM1->Set(0);
			shooterM2->Set(0);
			shooterIntake->Set(0);
			intakeShooterSet = false;
		}*/
//		barrel->Set(control_1->GetRawAxis(1));

/*
		shooterM1->Set(0.7);
		shooterM2->Set(-0.7);
*/

//		getPotAngle();

		lidar->reset();
		if(lidar->isMeasurementValid(false))
			SmartDashboard::PutNumber("lidar",lidar->getDistance()/2.54);
			printf("dist %f\n",lidar->getDistance()/2.54);

		motion_control->Manual_driving(control_0);
/*
		if(control_0->GetRawButton(1)){
			//intake->Set(-((-control_0->GetRawAxis(3)+1.0)/2.0));//with the throttle
			intake->Set(-0.75);
		}
		else if(control_0->GetRawButton(3)){
			intake->Set(0.75);
		}
		else{
			intake->Set(0);
		}

		if(control_0->GetRawButton(5))
			climber->Set(1);
		else if (control_1->GetRawButton(7))
			climber->Set(-1);
		else
			climber->Set(0);

		if(control_1->GetRawButton(6))
		{
			shooterIntake_Aux_Left->Set(1);
			shooterIntake_Aux_Right->Set(-1);
		}
		else if(control_1->GetRawButton(5))
		{
			shooterIntake_Aux_Left->Set(-1);
			shooterIntake_Aux_Right->Set(1);
		}
		else
		{
			shooterIntake_Aux_Left->Set(0);
			shooterIntake_Aux_Right->Set(0);
		}


		light->Set(Relay::Value::kOn);*/
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
