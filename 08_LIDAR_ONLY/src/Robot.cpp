#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "LidarLite.h"
#include <Wpilib.h>
#define LIDAR_I2C_DEFAULT_ADDR           0x62

class Robot: public frc::IterativeRobot {
public:

	LidarLite *lidar;
//	I2C* i2c_Lidar; //Declare i2c for Lidar
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
//		i2c_Lidar = new I2C(I2C::Port::kOnboard, LIDAR_I2C_DEFAULT_ADDR); //(I2C::Port::kOnboard or kMXP, Pixy Address)
		lidar = new LidarLite(I2C::Port::kOnboard,0X62);
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

		lidar->reset();
		if(lidar->isMeasurementValid(false))
//			SmartDashboard::PutNumber("lidar",lidar->getDistance()/2.54);
			printf("dist %f\n",lidar->getDistance()/2.54);
		else
			printf("INVALID MEASUREMENT %d\n",lidar->getDistance());

/*		i2c_Lidar->Write(0x00, 0x00);
	      Wait(.025);
	      i2c_Lidar->Write(0x00, 0x04);
	      Wait(.005);

	      unsigned char djoSendData[1];
	      unsigned char LidarMassData[2];

	      djoSendData[0] = 0x01;  // Bulk Write Based Read_Only for a single register Read of 0x01 (status)
	      i2c_Lidar->WriteBulk(djoSendData,1);
	      i2c_Lidar->ReadOnly(1,LidarMassData);
//	      printf("Mass Data 0 %4X \n", LidarMassData[0]);
	      Wait(0.001);

	      djoSendData[0] = 0x8F;  // Bulk Write Based Read_Only for a two register Read of 0x8F (distance)
	      i2c_Lidar->WriteBulk(djoSendData,1);
	      i2c_Lidar->ReadOnly(2,LidarMassData);
	      printf("Mass Data 0 %4X \t", LidarMassData[0]);
	      printf("Mass Data 1 %4X \t", LidarMassData[1]);
		  printf("dist: %d\n",(LidarMassData[0]<<8)+LidarMassData[1]);
*/

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
