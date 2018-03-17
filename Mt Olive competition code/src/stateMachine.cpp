/*
 * stateMachine.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: RoboWarriors
 */

/*for middle -
 * forawrds 10
 * turn 40 *c
 * forwards 80
 * turn back 40. as you lift.
 * shoot cube
 */

#include "stateMachine.h"

stateMachine::stateMachine(const char* gameSpecificMessage, char startSide, Driving *drvPointer, Lift *lftPointer){
	//map each event to the corresponding transition ....
	for(uint8_t i = 0; i < eventKeys.size();i++){
		event_to_transitions[eventKeys[i]] = (transitions)i;
	}
	drv = drvPointer;
	lft = lftPointer;
	eData = new eventData();
	eData->events[evtType::switchSide] = gameSpecificMessage[0] == startSide ? evtVal::IS : evtVal::IS_NOT;
	eData->events[evtType::scaleSide] = gameSpecificMessage[1] == startSide ? evtVal::IS : evtVal::IS_NOT;
	eData->events[evtType::currentState] = START;
	SmartDashboard::PutString("SM State", "CONSTRUCTOR");
	SmartDashboard::PutBoolean("switchSide",eData->events[evtType::switchSide]);
	SmartDashboard::PutBoolean("scaleSide",eData->events[evtType::scaleSide]);

	centerXPolarity = startSide == 'L'?1:-1;
	currentState = 0;
	currentTransition = 0;
}

void stateMachine::runState(){
	switch(eData->events[evtType::currentState]){
	case START:
		stateStart();
		break;
	case SWITCH:
		stateSwitch();
		break;
	case SCALE:
		stateScale();
		break;
	case PICKUP:
//		statePickup(); 	TODO:uncomment when done
		break;
	}
}

int stateMachine::runTransition(transitions t){
	switch(t){
	case DRIVE_FORWARD_SCALE:
		return driveForwardScale();
	case DRIVE_ACROSS_SWITCH:
		return driveAcrossSwitch();
	case DRIVE_FORWARD_SWITCH:
		return driveForwardSwitch();
	case SWITCH_TO_CUBE:
		return switchToCube();
	case SCALE_TO_CUBE:
		return scaleToCube();
	case CUBE_ACROSS_SWITCH:
		return cubeAcrossSwitch();
	case CUBE_TO_SWITCH:
		return cubeToSwitch();
	case CUBE_ACROSS_SCALE:
		return cubeAcrossScale();
	case CUBE_TO_SCALE:
		return cubeToScale();
	}
	return false;
}

int stateMachine::returnTransition(){
    for(auto it = event_to_transitions.cbegin(); it != event_to_transitions.cend(); ++it)
    {
    	bool allMatch = true;
 	   printf("events %d %d %d %d %d \n", it->first[0],it->first[1],it->first[2],it->first[3],it->first[4]);
 	   printf("datamu %d %d %d %d %d \n", eData->events[0],eData->events[1],eData->events[2],eData->events[3],eData->events[4]);

 	   for (int i = 0; i < 4; i++){
           //if (! ((it->first[i] == evtVal::DONT_CARE && i!=4) || it->first[i] == eData->events[i]) ){
    	   if (it->first[i] != eData->events[i] && it->first[i] != evtVal::DONT_CARE){
        	   allMatch = false;
        	   printf("failed on i: %d\n",i);
        	   break;
           }

       }
       if (it->first[4] != eData->events[4]) allMatch = false;
       if(allMatch){
    	   printf("success on: %d\n", it->first[4]);
    	   return it->second;
       }
   }
   return -1;
}

void stateMachine::stateStart(){
	SmartDashboard::PutString("SM State", "START");
	int returnState = runTransition((transitions)returnTransition());
	printf("returned state %d\n", returnState);
	if (IS_STATE(returnState)){
		eData->events[evtType::currentState] = (state)returnState;
	}
}

void stateMachine::stateSwitch(){
	//PLACE CUBE
//	if(cube placed){
//		eData->events[evtType::switchDone] = true;
//	}
	SmartDashboard::PutString("SM State", "SWITCH");
	switch(currentState){
/*	case 0:
		if(drv->AutoForwards(5))
			currentState++;
		break;*/
	case 0:
		printf("auto open claw\n");
		if(	lft->auto_claw_open()){
			//printf("claw off");
			currentState++;
			eData->events[evtType::switchDone] = true;
		}
		break;
	case 1:
		int returnState = runTransition((transitions)returnTransition());
		if (IS_STATE(returnState)){
			eData->events[evtType::currentState] = returnState;
		}
		currentState = 0;
	}
}

void stateMachine::stateScale(){
	SmartDashboard::PutString("SM State", "SCALE");
	switch(currentState){
	case 0:
		if(drv->AutoForwards(15) || frc::DriverStation::GetInstance().GetMatchTime() < 2.0)
			currentState++;
		break;
	case 1:
		printf("auto open claw\n");
		SmartDashboard::PutString("current state","auto open");
		if(	lft->auto_claw_open()){
			//printf("claw off");
			currentState++;
			eData->events[evtType::scaleDone] = true;
		}
		break;
	case 2:
		int returnState = runTransition((transitions)returnTransition());
		if (IS_STATE(returnState)){
			eData->events[evtType::currentState] = returnState;
		}
		currentState = 0;
	}
}

void stateMachine::statePickup(){
	SmartDashboard::PutString("SM State", "PICKUP");
	//PICKUP CUBE
	switch(currentState){
	case 0:
		if(drv->autoCube(lft))
			currentState++;
		break;
	case 1:
		if(drv->AutoForwards(22))
			currentState++;
		break;
	case 2:
		if(lft->auto_claw_clamp())
			currentState++;
		break;
	case 3:
		int returnState = runTransition((transitions)returnTransition());
		if (IS_STATE(returnState)){
			eData->events[evtType::currentState] = returnState;
		}
		currentState = 0;
	}

}
