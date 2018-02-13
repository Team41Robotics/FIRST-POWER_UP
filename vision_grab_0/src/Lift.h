/*
 * Lift.h
 *
 *  Created on: Feb 12, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_LIFT_H_
#define SRC_LIFT_H_
#include <WPILib.h>
#include "ctre/Phoenix.h"
#include "Driving.h"

#define MAX_LIFT_HEIGHT 0.0


class Lift {
private:
	TalonSRX * lift;
/*
 * 	clawLinear = new TalonSRX(9);					//claw linear x
		arm0 = new TalonSRX(3);						//intake
		arm1 = new TalonSRX(10);					//intake
		armAct = new TalonSRX(4);					//rotate claw up
		lift = new TalonSRX(8);						//lift to move vertically
 */
	TalonSRX * clawLinear;
	TalonSRX * arm0;
	TalonSRX * arm1;
	TalonSRX * armAct;

	DigitalInput * arm_lim_out;
	DigitalInput * arm_lim_in;
	Encoder * liftEncoder;
	DigitalInput * topLift;
	DigitalInput * bottomLift;
	double linearSpeed;

public:
	Lift();
	virtual ~Lift();

	bool reset();
	void sliderController(Joystick * driverstation);
};

#endif /* SRC_LIFT_H_ */
