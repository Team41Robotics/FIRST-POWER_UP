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
#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>
#include "WPILib_auxiliary.h"
//#include "LidarLite.h"

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
	CANTalon *climberM1;
	CANTalon *climberM2;

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
		shooterM2 = new CANTalon(8);

//		barrel = new CANTalon(5);
		shooterIntake = new CANTalon(1);

		intake = new CANTalon(10);

		climberM1 = new CANTalon(2);
		climberM2 = new CANTalon(5);

//		climber = new CANTalon(2);

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

		/*for red:
		 if(boiler){
		 	 if(auton_step == 0) {
		 	 	 drive forwards 99inches
		 	 }
		 	 else if(auton_step == 1) {
		 	 	 turn 120
		 	 }
		 	 else if(auton_step == 2) {
		 	 	 drive forwards 45.5 inches
		 	 }
		 }
		 if(key) {
		 	 if(auton_step == 0) {
				drive forwards 106 inches
			}
		 	else if(auton_step == 1) {
		 	 	turn -120
		 	}
		 	else if(auton_step == 2) {
		 		drive forwards 35 inches
			}
		 }

		for blue: need to mirror turn direction. distance is same.


		 */

//		motion_control->imu->Localization(motion_control->nav);
//		motion_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);


/*		if(timerauton->Get() < 3){
			motion_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);
		}
		else if (auton_step == 0){
			timerauton->Reset();
			if(motion_control->NOPID_Turn(-120.0,0.2))
				auton_step = 1;
		}
		else if (timerauton->Get() < 5){
			motion_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);
		}
		else if (timerauton->Get() < 7){
			motion_control->Auto_Move(0.28,0.28,0.28,0.28);
		}
		else if (timerauton->Get() < 10){
			motinn_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);
		}*/
/*
		if (timerauton->Get() < 2){
			motion_control->Auto_Move(-0.4,-0.4,-0.4,-0.4);
		}
		else if (timerauton->Get() < 8){
			motion_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);
		}
		else if (timerauton->Get() < 9){
			motion_control->Auto_Move(0.28,0.28,0.28,0.28);
		}
		else if (timerauton->Get() < 12){
			motion_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);
		}
		else if (timerauton->Get() < 13){
			motion_control->Auto_Move(0.28,0.28,0.28,0.28);
		}
		else if (timerauton->Get() < 15){
			motion_control->Auto_Move(-0.28,-0.28,-0.28,-0.28);
		}
*/

/*		if(auton_step == 0)
		{
			if(motion_control->NOPID_Turn(120.0,0.2)){
				auton_step = 1;
				motion_control->imu->Reset(motion_control->nav);
				Wait(1);
			}
		}
		else
		{
			motion_control->Auto_Stop();
		}*/

		if(autoSelected == test)
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
		if(colorSelected == red)
		{
			if(autoSelected == right)
			{
				printf("Going right\n");
				if(auton_step == 0)
				{
					if(motion_control->NOPID_Move(95-15,0.3)){
						auton_step = 1;
						Wait(0.3);
					}
				}
				else if(auton_step == 1)
				{
					if(motion_control->NOPID_Turn(-118.0,0.4)){
						auton_step = 2;
						motion_control->imu->Reset(motion_control->nav);
						Wait(0.3);
					}
				}
				else if(auton_step == 2)
				{
					if(motion_control->NOPID_Move(5.0,-0.12,false)){
						auton_step = 3;
						Wait(0.3);
					}
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
					if(motion_control->NOPID_Move(10.0,-0.2,false))
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
					if(motion_control->NOPID_Move(97-15,0.3))
						auton_step = 1;
				}
				else if(auton_step == 1)
				{
					if(motion_control->NOPID_Turn(-120.0,0.2)){
						auton_step = 2;
						motion_control->imu->Reset(motion_control->nav);
						Wait(1);
					}
				}
				else if(auton_step == 2)
				{
					if(motion_control->NOPID_Move(10.0,-0.2,false))
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
					if(motion_control->NOPID_Move(97,0.3))
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
					if(motion_control->NOPID_Move(10.0,-0.2,false))
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
					if(motion_control->NOPID_Move(10.0,-0.2))
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
					if(motion_control->NOPID_Move(103-15,0.3))
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
					if(motion_control->NOPID_Move(10.0,-0.2,false))
						auton_step = 3;
				}
				else
				{
					motion_control->Auto_Stop();
				}
			}
		}

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
//		printf("%f",motion_control->LidarDist());
		motion_control->LidarDist();
//		SmartDashboard::PutNumber("angle",motion_control->imu->theta);
		printf("imu: %f\t",motion_control->imu->theta);


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



		if(control_1->GetRawButton(13)){
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

		motion_control->Manual_driving(control_0);

		if(control_0->GetRawButton(1) || control_1->GetRawButton(6)){
			//intake->Set(-((-control_0->GetRawAxis(3)+1.0)/2.0));//with the throttle
			intake->Set(-0.75);
		}
		else if(control_1->GetRawButton(9)){
			intake->Set(0.75);
		}
		else{
			intake->Set(0);
		}

		if(control_1->GetRawButton(1)){
			climberM1->Set(1);
			climberM2->Set(1);
		}
		else{
			climberM1->Set(0);
			climberM2->Set(0);
		}
//		climberM1->Set(-((-control_1->GetRawAxis(3)+1.0)/2.0));
//		climberM2->Set(((-control_1->GetRawAxis(3)+1.0)/2.0));

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

/*		if(control_1->GetRawButton(10))
		{
			motion_control->NOPID_Move(13.5,0.2,false);
		}*/

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
