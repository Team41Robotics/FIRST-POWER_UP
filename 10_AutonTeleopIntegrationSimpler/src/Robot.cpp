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
//	CANTalon *barrel;
	CANTalon *shooterIntake;
	CANTalon *intake;
	CANTalon *climber;
	Servo *cam_servo;

	Talon *shooterIntake_Aux_Left;
	Talon *shooterIntake_Aux_Right;

	Timer *timer;
	Timer *timerauton;

	LidarLite *lidar;

	double cam_servo_angle = 0.0;
	double cam_servo_angle_adjust = 0.0;
	double cam_servo_tolerance = 0.0;

	bool intakeShooterSet;

	int auton_step = 0;
	bool stopped;

/*	double getPotAngle()
	{
		double read = pot->GetVoltage()-4.11;
		double angle = (read/0.7079)*300.0;//check scale again (TEST AGAIN)   4.813
		SmartDashboard::PutNumber("Raw pot", read);
		SmartDashboard::PutNumber("Pot Angle:", angle);
		return angle;
	}*/
	//	Relay *light;


	void RobotInit() {
		motion_control = new Driving();
		control_0 = new Joystick(0);
		control_1 = new Joystick(1);
		cam_servo = new Servo(2); /// pwm

		shooterIntake_Aux_Left = new Talon(0);// Servo(0);
		shooterIntake_Aux_Right = new Talon(1);// Servo(9);

		lidar = new LidarLite(I2C::Port::kOnboard,0X62);

		shooterM1 = new CANTalon(4);
		shooterM2 = new CANTalon(8);

//		barrel = new CANTalon(5);
		shooterIntake = new CANTalon(1);

		intake = new CANTalon(10);

		climber = new CANTalon(2);

//		pot = new AnalogInput(0);

		intakeShooterSet = false;
		timer = new Timer();
		timer->Reset();

		chooser.AddDefault(left, left);
		chooser.AddObject(middle, middle);
		chooser.AddObject(right, right);
		chooser.AddObject(test, test);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		color.AddDefault(blue, blue);
		color.AddObject(red, red);
		frc::SmartDashboard::PutData("color Modes", &color);

		timer = new Timer();
		timer->Start();
		timerauton = new Timer();
		timerauton->Start();

		stopped = false;

//		light = new Relay(0);
//		CameraServer *server = CameraServer::GetInstance();
//		server->StartAutomaticCapture(0);

//		CameraServer::GetInstance()->StartAutomaticCapture(0);
	}

	void AutonomousInit() override {

	}
	void AutonomousPeriodic() {
//		motion_control->imu->Localization(motion_control->nav);
		motion_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);

/*		if(autoSelected == test)
		{
			if(auton_step == 0)
			{
				if(motion_control->NOPID_Move(114.5,0.3))
					auton_step = 1;
			}
			else if(auton_step == 1)
			{
				if(motion_control->NOPID_Turn(120.0,0.2)){
					auton_step = 2;
					motion_control->imu->Reset(motion_control->nav);
					Wait(1);
				}
			}
			else if(auton_step == 2)
			{
				if(motion_control->NOPID_Move(32.0,-0.2,false))
					auton_step = 3;
			}
			else
			{
				motion_control->Auto_Stop();
			}
		}
		else if(colorSelected == red)
		{
			if(autoSelected == right)
			{
				if(auton_step == 0)
				{
					if(motion_control->Pid_Move_upTo(114.5))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->Pid_turn_downTo(-120.0))
						auton_step = 2;
				}
				else if(auton_step == 2)
				{
					if(motion_control->Pid_Move_downTo(10.0))
						auton_step = 3;
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
			else if (autoSelected == middle)
			{
				if(auton_step == 0)
				{
					if(motion_control->Pid_Move_downTo(10.0))
						auton_step = 1;
				}
				else if(auton_step ==1)
				{
					motion_control->Auto_Stop();
				}
			}
			else if (autoSelected == left)
			{
				if(auton_step == 0)
				{
					if(motion_control->Pid_Move_upTo(114.5))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->Pid_turn_upTo(120.0))
						auton_step = 2;
				}
				else if(auton_step == 2)
				{
					if(motion_control->Pid_Move_downTo(10.0))
						auton_step = 3;
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
		}
		else if (colorSelected == blue)
		{
			if(autoSelected == right)
			{
				if(auton_step == 0)
				{
					if(motion_control->Pid_Move_upTo(114.5))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->Pid_turn_upTo(120.0))
						auton_step = 2;
				}
				else if(auton_step == 2)
				{
					if(motion_control->Pid_Move_downTo(10.0))
						auton_step = 3;
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
			else if (autoSelected == middle)
			{
				if(auton_step == 0)
				{
					if(motion_control->Pid_Move_downTo(10.0))
						auton_step = 1;
				}
				else if(auton_step ==1)
				{
					motion_control->Auto_Stop();
				}
			}
			else if (autoSelected == left)
			{
				if(auton_step == 0)
				{
					if(motion_control->NOPID_Move(114.5,0.3))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->NOPID_Turn(-120.0,0.3))
						auton_step = 2;
				}
				else if(auton_step == 2)
				{
					if(motion_control->NOPID_Move(20.0,-0.3,false))
						auton_step = 3;
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
		}
*/
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

		//servo:
		//get servo value somewhere; Possibly user input or compute. For this it will grab from smartdash
		cam_servo->SetAngle(cam_servo_angle);

#ifdef CAM_SERVO_ON

		printf("%d",control_1->GetPOVCount());
		if(control_1->GetPOV() == 0){
			cam_servo_angle = 90;// + cam_servo_angle_adjust; //so it returns the proper angle from our chosen reference frame
			cam_servo->SetAngle(cam_servo_angle);}
		else if(control_1->GetPOV() == 180){
			cam_servo_angle = 0;// + cam_servo_angle_adjust; //so it returns the proper angle from our chosen reference frame
			cam_servo->SetAngle(cam_servo_angle);
		}

/*		if(fabs(cam_servo->GetAngle()-cam_servo_angle) <= cam_servo_tolerance) {
			SmartDashboard::PutNumber("On Angle",true);
		} else {
			SmartDashboard::PutNumber("On Angle",false);
		}*/
#endif



		if(control_1->GetRawButton(1)){
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
	 	}
		else{
			timer->Reset();
			shooterM1->Set(0);
			shooterM2->Set(0);
			shooterIntake->Set(0);
			intakeShooterSet = false;
		}
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

		if(control_1->GetRawButton(4))
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

		if(control_1->GetRawButton(10))
		{
			motion_control->NOPID_Move(32.0,-0.2,false);
		}


//		light->Set(Relay::Value::kOn);
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string left = "Left";
	const std::string middle = "Middle";
	const std::string right = "Right";
	const std::string test = "Test";



	frc::SendableChooser<std::string> color;
	const std::string blue = "Blue";
	const std::string red = "Red";


	std::string autoSelected;
	std::string colorSelected;
};

START_ROBOT_CLASS(Robot)
