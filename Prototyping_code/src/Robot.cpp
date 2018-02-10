#include "WPILib.h"
#include "stdio.h"
#include "CANTalon.h"
using namespace std;
#include <vector>

class Robot: public IterativeRobot
{
private:

public:

	LiveWindow *lw = LiveWindow::GetInstance();
	Command *autonCommand;

	struct Motor {
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

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
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
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
