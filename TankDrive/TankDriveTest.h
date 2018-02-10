/*
 * TankDriveTest.h
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#ifndef TANKDRIVETEST_H_
#define TANKDRIVETEST_H_
#include <WPILib.h>

class TankDriveTest {
private:
    CANTalon *leftTalon0;
    CANTalon *rightTalon0;
    CANTalon *leftTalon1;
    CANTalon *rightTalon1;
bool stop;
public:
    Driving();
void move(double leftSpeed, double rightSpeed);
    void stop();

};

#endif /* TANKDRIVETEST_H_ */
