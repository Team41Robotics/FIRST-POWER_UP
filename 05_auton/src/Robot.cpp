#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Driving.h>
#include <WPILib_auxiliary.h>
#include <Shooter.h>

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Shooter *shooter;
	Joystick *control_0;
	Timer *timer;

	double time = 0.0;
	int auton_step = 0;
	bool stopped;


	void RobotInit() {
		chooser.AddDefault(left, left);
		chooser.AddObject(middle, middle);
		chooser.AddObject(right, right);
		chooser.AddObject(test, test);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		color.AddDefault(blue, blue);
		color.AddObject(red, red);
		frc::SmartDashboard::PutData("color Modes", &color);


		motion_control = new Driving();
		shooter = new Shooter();
		control_0 = new Joystick(0);
		timer = new Timer();
		timer->Start();
		stopped = false;
	}

	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (colorSelected == left) {
			SmartDashboard::PutString("Auton","left");
		}
		else if (autoSelected == middle) {
			SmartDashboard::PutString("Auton","middle");
		}
		else if (autoSelected == right){
			SmartDashboard::PutString("Auton","right");
		}

		colorSelected = color.GetSelected();
		std::cout << "Color selected: " << colorSelected << std::endl;

		if (colorSelected == blue) {
			SmartDashboard::PutString("Color","Blue");
		}
		else if (colorSelected == red) {
			SmartDashboard::PutString("Color","Red");
		}


		timer->Reset();
		timer->Start();
		auton_step = 0;
		motion_control->imu->Reset(motion_control->nav);
	}
	void AutonomousPeriodic() {
		time = timer->Get();
		motion_control->imu->Localization(motion_control->nav);

		if(autoSelected == test)
		{
			if(auton_step == 0)
			{
				if(motion_control->NOPID_Move(114.5,0.3))
					auton_step = 1;
			}
			else if(auton_step == 1)
			{
				if(motion_control->NOPID_Turn(-120.0,0.2)){
					auton_step = 2;
					motion_control->imu->Reset(motion_control->nav);
					Wait(2);
				}
			}
			else if(auton_step == 2)
			{
				if(motion_control->NOPID_Move(32.0,-0.2,false))
					auton_step = 4;
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



	/* else {
			time = timer->Get();
			imu->Localization(nav);
			SmartDashboard::PutNumber("distance",imu->position_y);
			if(imu->position_y < 1.00)
			{

				motion_control->Auto_Move(-0.5,-0.5,-0.5,-0.5);
			}
		}*///IN REFERNCE TO THIS TAPE...
	}

	void TeleopInit() {
		motion_control->imu->Reset(motion_control->nav);
		auton_step = 0;
	}

	void TeleopPeriodic() {
		motion_control->Manual_driving(control_0);
		motion_control->imu->Localization(motion_control->nav);

		printf("ang %f \n",motion_control->imu->theta);
/*		motion_control->imu->Localization(motion_control->nav);
		if(auton_step == 0)
		{
			if(motion_control->NOPID_Move(90.0,0.3))
				auton_step = 1;
		}
		else if(auton_step == 1)
		{

			if(motion_control->NOPID_Turn(-120.0,0.4))
				auton_step = 2;
		}*/
/*		motion_control->imu->Localization(motion_control->nav);
		//motion_control->Manual_driving(control_0);
		SmartDashboard::PutNumber("smaller lazer-y prateek. ",motion_control->LidarDist());
		SmartDashboard::PutNumber("smaller gyro-y prateek. ", motion_control->imu->theta);
		SmartDashboard::PutNumber("Smaller displacement-y prateek",motion_control->nav->GetDisplacementY()*100);
		if(control_0->GetRawButton(12))
			motion_control->imu->Reset(motion_control->nav);

		SmartDashboard::PutNumber("vel Y",motion_control->nav->GetVelocityY());

		printf("vel pos: %f\n",motion_control->imu->position_y*100/2.54);
		if(!stopped){
			if(fabs(motion_control->imu->position_y*100/2.54) < 30.0){
				motion_control->Auto_Foward(1);
				stopped = true;
			}
			else
				motion_control->Auto_Foward(0.0);
		}
		//		if(motion_control->nav->GetVelocityY()*timer->Get())

		//printf("a shirt but slightly smaller %f\n",motion_control->imu->theta);
/*

		imu->Localization(nav);

		SmartDashboard::PutNumber("raw Accel",nav->GetRawAccelY());
		SmartDashboard::PutNumber("raw Accelx",nav->GetRawAccelX());
		SmartDashboard::PutNumber("raw gyroz",nav->GetRawGyroZ());
		SmartDashboard::PutNumber("raw gyrox",nav->GetRawGyroX());
		SmartDashboard::PutNumber("raw gyroy",nav->GetRawGyroY());
		SmartDashboard::PutNumber("velocity ",nav->GetVelocityY());

		SmartDashboard::PutNumber("X", imu->position_x);
		SmartDashboard::PutNumber("Y", imu->position_y);
		SmartDashboard::PutNumber("vel x", imu->velocity_x);
		SmartDashboard::PutNumber("vel Y", imu->velocity_y);
		SmartDashboard::PutNumber("accel Y", imu->accel_y);
		SmartDashboard::PutNumber("accel X", imu->accel_x);

		//SmartDashboard::PutNumber("D Y", nav->GetDisplacementX());

		if(control_0->GetRawButton(12))
		{
			imu->Reset(nav);
		}
		//imu->Localization(nav);
		//SmartDashboard::PutNumber("X",imu->position_x);
		//SmartDashboard::PutNumber("Y",imu->position_y);
		//SmartDashboard::PutNumber("R",imu->position_r);*/
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
