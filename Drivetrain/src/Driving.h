/*
 * driving.h
 *
 *  Created on: Feb 2, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_
#include "ctre/Phoenix.h"
#include <WPILib.h>

class Driving {
private:
	Encoder *enc;
	TalonSRX *leftBack;
	TalonSRX *leftFront;
	TalonSRX *rightBack;
	TalonSRX *rightFront;
public:
	Driving();
	void move(double leftSpeed, double rightSpeed);
	void stop();
};

#endif /* SRC_DRIVING_H_ */
