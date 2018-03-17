/*
 * transitions.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: RoboWarriors
 */

#include "stateMachine.h"

int stateMachine::driveForwardScale(){
	SmartDashboard::PutString("SM Transition", "DRIVE FORWARD SCALE");
	//drive forward and turn towards center x and drop the cube
	switch(currentTransition){
	case 0:
			lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoForwards(249) ){//249
				currentTransition++;
				printf("added one\n");
			} else //printf("No addy\n");
			SmartDashboard::PutString("current state","driving forward");

		break;
/*	case 1:
		if(lft->PIDLift(lft->SET_HEIGHTS::SCALE)){
			currentTransition++;
		}
		break;*/
	case 1:
		//lft->PIDLift(lft->SET_HEIGHTS::SCALE);
		if(drv->AutoTurn(27.0*centerXPolarity, 4.0))
			currentTransition++;
		else printf("still turning");
		SmartDashboard::PutString("current state","turn");
		break;
	case 2:
		currentTransition = 0;
		return SCALE;
	}
	return -1;
}

int stateMachine::driveAcrossSwitch(){
	SmartDashboard::PutString("SM Transition", "DRIVE ACROSS SWITCH");
	//drive around the switch:
	//forward-> turn to center x -> drive forward -> turn towards switch when inbetween cubes -> drive forward and drop
	switch(currentTransition){
		case 0:
			//lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoForwards(235 - 12) ) currentTransition++;
			SmartDashboard::PutString("current state","drive forward past switch");
			break;
		case 1:
			//lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoTurn(90.0*centerXPolarity)) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 to center");
			break;
		case 2:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoForwards(174)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards to switch");
			break;
		case 3:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoTurn(90.0*centerXPolarity)) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 to switch");
			break;
/*		case 4:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoForwards(50)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards to switch");
			break;*/
		case 4:
			currentTransition = 0;
			return SWITCH;
		}
	return -1;
}

int stateMachine::driveForwardSwitch(){
	SmartDashboard::PutString("SM Transition", "DRIVE FORWARD SWITCH");
	//drive forward to switch
	switch(currentTransition){
		case 0:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoForwards(156) ) currentTransition++; //168
			SmartDashboard::PutString("current state","driving forward to switch");
			break;
		case 1:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoTurn(90.0*centerXPolarity)) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 to switch");
			break;
		case 2:
			currentTransition = 0;
			return SWITCH;
		}
	return -1;
}

/*
 * probably better than going across is to just to spit two in one switch
 */

int stateMachine::switchToCube(){
	SmartDashboard::PutString("SM Transition", "SWITCH TO CUBE");
	//backup from switch turn towards center go fowards to cube turn pick up with vision and back up
	switch(currentTransition){
		case 0:

			lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoTurn(-90.0*centerXPolarity)) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 away from center");
			break;
		case 1:
			lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoForwards(74)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards past switch");
			break;
		case 2:
			lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoTurn(90.0*centerXPolarity)) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 to center");
			break;
		case 3:
			lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoForwards(54)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards perpendicular to cubes");
			break;
		case 4:
			lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoTurn(90.0*centerXPolarity)) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 to cubes");
			break;
		case 5:
			lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoForwards(17)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards to cubes");
			break;
		case 6:
			currentTransition = 0;
			return PICKUP;
		}
	return -1;
}

int stateMachine::scaleToCube(){
	SmartDashboard::PutString("SM Transition", "SCALE TO CUBE");
	//back up from the scale turn towards center drive forward to the closest cube and drive forward to pick it up
	switch(currentTransition){
		case 0:
			//lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoTurn((90.0+18)*centerXPolarity)) currentTransition = 0;
			SmartDashboard::PutString("current state","turning 90 towards cube");
			break;
		case 1:
			lft->PIDLift(lft->SET_HEIGHTS::EXCHANGE);
			if(drv->AutoForwards(50) ) currentTransition++;
			SmartDashboard::PutString("current state","forwards perpendicular to cubes");
			break;
		case 2:
			lft->PIDLift(lft->SET_HEIGHTS::BOTTOM);
			if(drv->AutoTurn(45*centerXPolarity)) currentTransition++;
			SmartDashboard::PutString("current state","turning 45 to cubes");
			break;
		case 3:
			currentTransition = 0;
			return PICKUP;
		}
	return -1;
}

int stateMachine::cubeAcrossSwitch(){
	SmartDashboard::PutString("SM Transition", "CUBE ACROSS SWITCH");
	//back up turn towards center and drive across to the other side of switch and then turn towards the switch and go forward in between cubes
	switch(currentTransition){
		case 0:
			//lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoForwards(-50)) currentTransition++;
			SmartDashboard::PutString("current state","backwards");
			break;
		case 1:
			//lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoTurn(-90.0*centerXPolarity) ) currentTransition++;
			SmartDashboard::PutString("current state","turn -90 away from cubes");
			break;
		case 2:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoForwards(174)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards");
			break;
		case 3:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoTurn(90.0*centerXPolarity) ) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 to switch");
			break;
/*		case 4:
			lft->PIDLift(lft->SET_HEIGHTS::SWITCH);
			if(drv->AutoForwards(20)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards");
			break;*/
		case 4:
			currentTransition = 0;
			return SWITCH;
		}
	return -1;
}

int stateMachine::cubeToSwitch(){
	SmartDashboard::PutString("SM Transition", "CUBE TO SWITCH");
	//raise claw and drive forward a little
	switch(currentTransition){
		case 0:
			if(lft->PIDLift(lft->SET_HEIGHTS::SWITCH)) currentTransition++;
			SmartDashboard::PutString("current state","raise lift to switch");
			break;
		case 1:
			currentTransition = 0;
			return SWITCH;
		}
	return -1;
}

int stateMachine::cubeAcrossScale(){
	SmartDashboard::PutString("SM Transition", "CUBE ACROSS SCALE");
	//back up turn to center and drive towards the scale
	switch(currentTransition){
		case 0:
			//lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoForwards(-50)) currentTransition++;
			SmartDashboard::PutString("current state","backwards");
			break;
		case 1:
			//lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoTurn(-90.0*centerXPolarity) ) currentTransition++;
			SmartDashboard::PutString("current state","turn -90 away from cubes");
			break;
		case 2:
			lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoForwards(174)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards");
			break;
		case 3:
			lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoTurn(-90.0*centerXPolarity) ) currentTransition++;
			SmartDashboard::PutString("current state","turn 90 to scale");
			break;
		case 4:
			lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoForwards(20)) currentTransition++;
			SmartDashboard::PutString("current state","drive forwards");
			break;
		case 5:
			currentTransition = 0;
			return SCALE;
		}
	return -1;
}

int stateMachine::cubeToScale(){
	SmartDashboard::PutString("SM Transition", "CUBE TO SCALE");
	//back up turn away from center drive forward turn towards center y and drive forward to the scale
	switch(currentTransition){
		case 0:
			lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoForwards(-55)) currentTransition++;
			SmartDashboard::PutString("current state","backwards");
			break;
		case 1:
			lft->PIDLift(lft->SET_HEIGHTS::SCALE);
			if(drv->AutoTurn(-180.0*centerXPolarity) ) currentTransition++;
			SmartDashboard::PutString("current state","turn -180 away from cubes");
			break;
		case 5:
			currentTransition = 0;
			return SCALE;
		}
	return -1;
}

