/*
 * Pneumatics.h
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_PNEUMATICS_H_
#define SRC_PNEUMATICS_H_
#include <WPILib.h>
#include "WPILib_auxiliary.h"

class Pneumatics {
public:
	Pneumatics();
	void runCatapult();
	void runFan();
	void catapultDown();
	DoubleSolenoid *rollerSolenoid;
	DoubleSolenoid *shooterSolenoid;
	virtual ~Pneumatics();
	Solenoid *fan;
	void catapult(Joystick *joy);
};

#endif /* SRC_PNEUMATICS_H_ */
