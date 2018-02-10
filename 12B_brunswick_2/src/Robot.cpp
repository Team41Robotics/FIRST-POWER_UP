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
#include "WPILib_auxiliary.h"
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include "LidarLite.h"

//#define CAM_SERVO_ON

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Joystick *control_0;
	Joystick *control_1;

	CANTalon *shooterM1;
//	CANTalon *shooterM2;
//	CANTalon *barrel;
	CANTalon *shooterIntake;
	CANTalon *intake;
	CANTalon *climberM1;
	CANTalon *climberM2;
	CANTalon *climberM3;

	Servo *cam_servo;

	Talon *shooterIntake_Aux_Left;
	Talon *shooterIntake_Aux_Right;

	Timer *timer;
	Timer *timerauton;

	Ultrasonic *sonar;
	DigitalInput *gear_trigger;

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

		//sonar = new Ultrasonic(9,8);
		//gear_trigger = new DigitalInput(9);

		shooterM1 = new CANTalon(4);
//		shooterM2 = new CANTalon(8);

//		barrel = new CANTalon(5);
		shooterIntake = new CANTalon(1);

		intake = new CANTalon(10);

		climberM1 = new CANTalon(2);
		climberM2 = new CANTalon(5);
		climberM3 = new CANTalon(8);

//		climber = new CANTalon(2);

//		pot = new AnalogInput(0);

		intakeShooterSet = false;
		timer = new Timer();
		timer->Reset();

		chooser.AddDefault(left, left);
		chooser.AddObject(middle, middle);
		chooser.AddObject(right, right);
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
		motion_control->imu->Reset(motion_control->nav);

	}

	void AutonomousInit() override {
		motion_control->imu->Reset(motion_control->nav);
		motion_control->lidar->reset();
		timerauton->Reset();
		timerauton->Start();
		autoSelected = chooser.GetSelected();
		colorSelected = color.GetSelected();
		auton_step = 0;

	}
	void AutonomousPeriodic() {

		motion_control->imu->Localization(motion_control->nav);

/*		if (timerauton->Get() < 2){
			motion_control->Auto_Move(-0.14,-0.14,-0.14,-0.14);
		}
		else if (timerauton->Get() < 8){
			motion_control->Auto_Move(-0.12,-0.12,-0.12,-0.12);
		}
		else if (timerauton->Get() < 9){
			motion_control->Auto_Move(0.12,0.12,0.12,0.12);
		}
		else if (timerauton->Get() < 12){
			motion_control->Auto_Move(-0.12,-0.12,-0.12,-0.12);
		}
		else if (timerauton->Get() < 13){
			motion_control->Auto_Move(0.12,0.12,0.12,0.12);
		}
		else if (timerauton->Get() < 15){
			motion_control->Auto_Move(-0.12,-0.12,-0.12,-0.12);
		}
*/
/*		if(colorSelected == red)
		{
			if(autoSelected == right)
			{
				printf("Going right\n");
				if(auton_step == 0)
				{
//					if(motion_control->NOPID_Move(101.75-15.0,0.3)){
					if(motion_control->NOPID_Move(86.75-15.0,0.3)){
						auton_step = 1;
						Wait(0.3);
					}
				}
				else if(auton_step == 1)
				{
					if(motion_control->NOPID_Turn(116.0,0.4)){
						auton_step = 2;
						motion_control->imu->Reset(motion_control->nav);
						Wait(0.3);
					}
				}
				else if(auton_step == 2)
				{
					motion_control->Auto_Foward(-0.12);
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
			else if (autoSelected == middle)
			{
				if ( auton_step == 0)
				{
					motion_control->Auto_Foward(-0.12);
				}
				else if ( auton_step ==1)
				{
					motion_control->Auto_Stop();
				}
			}
			else if (autoSelected == left)
			{
				if(auton_step == 0)
				{
					if(motion_control->NOPID_Move(93.7-15.0,0.3))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->NOPID_Turn(-116.0,0.4)){
						auton_step = 2;
						motion_control->imu->Reset(motion_control->nav);
						Wait(1);
					}
				}
				else if(auton_step == 2)
				{
					motion_control->Auto_Foward(-0.18);
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
					if(motion_control->NOPID_Move(93.7-15.0,0.3))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->NOPID_Turn(116.0,0.4)){
						auton_step = 2;
						motion_control->imu->Reset(motion_control->nav);
						Wait(1);
					}
				}
				else if(auton_step == 2)
				{
					motion_control->Auto_Foward(-0.18);
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
			else if (autoSelected == middle)
			{
				if ( auton_step == 0)
				{
					motion_control->Auto_Foward(-0.12);
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
					if(motion_control->NOPID_Move(86.75-15.0,0.3))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->NOPID_Turn(-116.0,0.4)){
						auton_step = 2;
						motion_control->imu->Reset(motion_control->nav);
						Wait(1);
					}
				}
				else if(auton_step == 2)
				{
					motion_control->Auto_Foward(-0.12);
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
		}*/
		/*
		if(auton_step == 0)
		{
			if(motion_control->NOPID_Move(93.3-15.0,0.3))
				auton_step = 1;
		}
		else if(auton_step == 1)
		{
			if(motion_control->NOPID_Turn(-116.0,0.4)){
				auton_step = 2;
				motion_control->imu->Reset(motion_control->nav);
				Wait(1);
			}
		}
		else if(auton_step == 2)
		{
			motion_control->Auto_Foward(-0.18);
		}
		else
		{
			motion_control->Auto_Stop();
		}*/
		motion_control->Auto_Foward(0.6);
		Wait(3);
		motion_control->Auto_Foward(0.0);


	}

	void TeleopInit() {
		motion_control->imu->Reset(motion_control->nav);
		motion_control->lidar->reset();
	}

	void TeleopPeriodic() {

		//servo:
		//get servo value somewhere; Possibly user input or compute. For this it will grab from smartdash
//		cam_servo->SetAngle(cam_servo_angle);
		motion_control->imu->Localization(motion_control->nav);

//		SmartDashboard::PutNumber("angle",motion_control->imu->theta);
		printf("imu: %f\t",motion_control->imu->theta);
		motion_control->LidarDist();
//		for(int i = 0; i <= 8; i++)
//			printf("%d: %f\t",i,control_1->GetRawAxis(i));
//		printf("\n");
//		printf("%f",(((-control_1->GetRawAxis(3)+1.0)/2.0)*(1.0/0.700787))-0.426967);
		//for sonar
		//printf("sonar reading... %f\n",sonar->GetRangeInches());
		//for gear trigger
//		printf("gear trigger is reading... %f\n",gear_trigger->Get());


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


		int polarity = control_1->GetRawButton(20) == 1?-1:1;
		if(control_1->GetRawButton(13)){
			float shooterSpeed = (((-control_1->GetRawAxis(4)+1.0)/2.0)*(1.0/0.700787))-0.426967;
			shooterM1->Set(shooterSpeed);
//			shooterM2->Set(((-control_1->GetRawAxis(3)+1.0)/2.0));
			printf("%f\n",(shooterSpeed));
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
//			shooterM2->Set(0);
			shooterIntake->Set(0);
			intakeShooterSet = false;
		}
//		barrel->Set(control_1->GetRawAxis(1));

/*
		shooterM1->Set(0.7);
		shooterM2->Set(-0.7);
*/
//		getPotAngle();

		motion_control->Manual_driving(control_0);

		if(control_0->GetRawButton(1) || control_1->GetRawButton(6)){
			//intake->Set(-((-control_0->GetRawAxis(3)+1.0)/2.0));//with the throttle
			intake->Set(-0.75*polarity);
		}
		else{
			intake->Set(0);
		}

		if(control_1->GetRawButton(1)){
			float climbSpeed = (((-control_1->GetRawAxis(3)+1.0)/2.0)*(1.0/0.700787))-0.426967;
			climberM1->Set(climbSpeed);
			climberM2->Set(climbSpeed);
			climberM3->Set(climbSpeed);
		}

		else if(control_1->GetRawButton(19) && control_1->GetRawButton(17)){
			climberM1->Set(-1);
			climberM2->Set(-1);
			climberM3->Set(-1);
		}
		else{
			climberM1->Set(0);

			climberM2->Set(0);
			climberM3->Set(0);
		}

		if(control_1->GetRawButton(21) == 1){
			if(control_1->GetRawButton(3))
			{
				shooterIntake_Aux_Left->Set(1);
				shooterIntake_Aux_Right->Set(-1);
			}
			else if(control_1->GetRawButton(4))
			{
				shooterIntake_Aux_Left->Set(-1);
				shooterIntake_Aux_Right->Set(1);
			}
			else
			{
				shooterIntake_Aux_Left->Set(0);
				shooterIntake_Aux_Right->Set(0);
			}
		}
		else{
			if(control_1->GetRawButton(3))
			{
				shooterIntake_Aux_Left->Set(1);
				shooterIntake_Aux_Right->Set(1);
			}
			else if(control_1->GetRawButton(4))
			{
				shooterIntake_Aux_Left->Set(1);
				shooterIntake_Aux_Right->Set(1);
			}
			else
			{
				shooterIntake_Aux_Left->Set(0);
				shooterIntake_Aux_Right->Set(0);
			}
		}

		if(control_1->GetRawButton(10))
		{
			motion_control->NOPID_Move(13.5,0.2,false);
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



	frc::SendableChooser<std::string> color;
	const std::string blue = "Blue";
	const std::string red = "Red";


	std::string autoSelected;
	std::string colorSelected;
};

START_ROBOT_CLASS(Robot)
