#include <AnalogPotentiometer.h>
#include <CANTalon.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotBase.h>
#include <stdio.h>
#include <cmath>
#include <AnalogOutput.h>
#include <Counter.h>
#include <WPILib.h>

using namespace std;
#include <vector>
#define PI 3.1415926535

class Robot: public IterativeRobot
{
private:

public:

	LiveWindow *lw = LiveWindow::GetInstance();
	Command *autonCommand;
	CANTalon *shooterM1;
	CANTalon *shooterM2;
	CANTalon *barrel;
	Joystick *control_0;
	CANTalon *john;
	CANTalon *testMotor;
	PWM *lidar;
//	DigitalOutput *lidarTrigger;
	AnalogInput *pot;
	AnalogOutput *led;
	AnalogOutput *led2;

	Relay *strip;
	Counter *test_boy;

	//digitalOut *trigger;

/*	struct Motor {
		CANTalon *motorController;
		float speed = 0;
	};

	std::vector<Motor> currMotors;


	void Add_Motor(int port){
		Motor newMotor;
		//newMotor.motorController = new CANTalon(port);
		SmartDashboard::PutNumber("Motor Speed (from -1.0 to 1.0):", newMotor.speed);
		currMotors.push_back(newMotor);
	}
*/
	void RobotInit()
	{
		test_boy = new Counter(0);
	}
	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		shooterM1 = new CANTalon(4);
		shooterM2 = new CANTalon(8);

		barrel = new CANTalon(5);
		control_0 = new Joystick(0);

		john = new CANTalon(1);
		pot = new AnalogInput(3);

		led= new AnalogOutput(0);
		led2= new AnalogOutput(1);

//		lidar = new PWM(0);
//		lidarTrigger = new DigitalOutput(1);
//		lidarTrigger->Set(false);
		testMotor = new CANTalon(1);
		strip = new Relay(0);
	}

	void TeleopPeriodic()
	{

		//lidar->GetRaw()
//		testMotor->Set(control_0->GetRawAxis(1));
		led->SetVoltage(5.0);
		led2->SetVoltage(5.0);
		printf("%f",led->GetVoltage());
		strip->Set();
//		getPotAngle();
//		getPotAngle();
		//shooterM1->Set(-((-control_0->GetRawAxis(3)+1.0)/2.0));
		//shooterM2->Set(((-control_0->GetRawAxis(3)+1.0)/2.0));
		//1 monitr
		//0 trig
		//SmartDashboard::PutNumber("period",test_boy->GetPeriod());

//		printf("period %f\n",test_boy->GetPeriod());

		//printf("input %f\n", 2.0*acos(control_0->GetRawAxis(1))/PI-1.0 );
		//printf("input %f : real %f\n", pow(-control_0->GetRawAxis(1),2.2) ,-control_0->GetRawAxis(1));
//		double pot_theta = pot -> Get();
	//	printf("Pot: %f\n", pot_theta);

//		testMotor->Set(0.8);
//john->Set(1.0);
	//	barrel->Set(-control_0->GetRawAxis(1));
	/*
		printf("started...");
		double port = SmartDashboard::GetNumber("Add Motor with port (from 0 to 4):",-1.0);
		printf("port: %f",port);
		if(port != -1.0){
			Add_Motor(port);
		}
		SmartDashboard::PutNumber("Add Motor with port (from 0 to 4):",-1);

		for(int i = 0; i < currMotors.size(); i++){
			currMotors[i].speed= SmartDashboard::GetNumber("Motor Speed (from -1.0 to 1.0):",currMotors[i].speed);
			currMotors[i].motorController->Set(currMotors[i].speed);
		}*/
	}

	void TestPeriodic()
	{
		lw->Run();
	}
	double getPotAngle()
	{
		double read = pot->GetVoltage()-4.11;
		double angle = (read/0.7079)*300.0;//check scale again (TEST AGAIN)   4.813
		SmartDashboard::PutNumber("Raw pot", read);
		SmartDashboard::PutNumber("Pot Angle:", angle);
		return angle;
	}
};

START_ROBOT_CLASS(Robot)
