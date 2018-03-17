#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "Driving.h"
#include "Lift.h"
#include "StateMachine.h"

class Robot : public frc::IterativeRobot {
private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string left = "left";
	const std::string middle = "middle";
	const std::string right = "right";
	std::string m_autoSelected;
	std::string gameMessage;
	//char gameMessage[3] = {'L','L','L'};

	Driving *driving;
	Lift *lift;
	stateMachine *SM;
	int phase = 0;
	Joystick *joyLeft;
	Joystick *joyRight;
	Joystick *buttonBoard;

public:
	void RobotInit() {
		m_chooser.AddDefault(left, left);
		m_chooser.AddObject(middle, middle);
		m_chooser.AddObject(right, right);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);//put the sendable chooser options in the smartdashboard
		driving = new Driving();
		lift = new Lift();
		joyLeft = new Joystick(2);
		joyRight = new Joystick(1);
		buttonBoard = new Joystick(0);

	}

	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		//m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault); get SmartDashboard data
		char initPos;
		if (m_autoSelected == middle) {
			initPos = 'M';
		} else if (m_autoSelected == right){
			initPos = 'R';
		}
		else{
			initPos = 'L';
		}
		if (SM != NULL)
			delete SM;
		gameMessage = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	//	gameMessage = "RRR";
		SM = new stateMachine(gameMessage.c_str(), initPos, driving, lift);
		driving->resetEncoders();
		driving->wharhs->Reset();
		lift->reset();

	}

	void AutonomousPeriodic() {
		driving->updatePosition();
		SM->runState();
		//driving->AutoTurn(-90);
		//driving->AutoForwards(50);

		SmartDashboard::PutNumber("integral",driving->integral );
		SmartDashboard::PutNumber("rEnc", driving->pos.x);
		SmartDashboard::PutNumber("lEnc", driving->pos.y);
		SmartDashboard::PutNumber("speed", driving->speed);
		SmartDashboard::PutNumber("angle", driving->wharhs->GetAngle());
		SmartDashboard::PutNumber("int_angle", driving->integral_angle);
		SmartDashboard::PutNumber("error_angle", driving->error_angle);
		SmartDashboard::PutNumber("error forward", driving->error);
	}

	void TeleopInit()
	{
		driving->resetEncoders();
		driving->wharhs->Reset();
		//lift->reset();
	}

	void TeleopPeriodic() {
		driving->updatePosition();
		//driving->AutoForwards(35);


		driving->tankDrive(joyLeft,joyRight);
		lift->runLift(joyLeft,joyRight,buttonBoard);
		/*switch(phase){
		case 0:
			if(driving->autoCube(lift))
				phase++;
			break;
		case 1:
			if(driving->AutoForwards(20))
				phase++;
			break;
		case 2:
			if(lift->auto_claw_clamp())
				phase++;
			break;
		}*/

		SmartDashboard::PutNumber("claw pot", lift->clawPot->GetValue());

		//driving->AutoTurn(90);
		SmartDashboard::PutNumber("integral", driving->integral);
		SmartDashboard::PutNumber("rEnc", driving->right_encoder);
		SmartDashboard::PutNumber("lEnc", driving->left_encoder);
		SmartDashboard::PutNumber("speed", driving->speed);
		SmartDashboard::PutNumber("angle", driving->wharhs->GetAngle());
		SmartDashboard::PutNumber("int_angle", driving->integral_angle);
		SmartDashboard::PutNumber("error_angle", driving->error_angle);


		SmartDashboard::PutNumber("lift encoder", -lift->lift->GetSensorCollection().GetQuadraturePosition());


	}

	void TestPeriodic() {

	}

};

START_ROBOT_CLASS(Robot)
