/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <memory>
#include <string>

#include <WPIlib.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include "Driving.h"
#include <AHRS.h>
#include "PID.h"

class Robot : public frc::IterativeRobot {
public:

		Driving *driving;
	    PID *pid;
	    int autonPlan;
	    int autonStage;
	    Joystick *joy1;
	   	Joystick *joy2;
	   	Encoder *enc;
	   	ADXL362 * test;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		joy1 = new Joystick(1);
		joy2 = new Joystick(0);
		driving = new Driving();
	//	pid = new PID();
		enc = new Encoder(0, 0);
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
	    void AutonomousInit() override {//<<s e a n>>
	        autoSelected = chooser.GetSelected();
	        // std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
	        std::cout << "Auto selected: " << autoSelected << std::endl;

	        if (autoSelected == autoNameCustom) {
	            // Custom Auto goes here
	        } else {
	            // Default Auto goes here
	            int startPos = 1; //1 is left, 2 is middle, 3 is right
	            autonStage = 1;
	            bool switchLeft = true;
	            bool scaleLeft = true;
	            if (startPos == 1) {
	                if (switchLeft || scaleLeft){
	                    if (switchLeft){
	                        if (scaleLeft) { //Starting at left, switch and scale are left
	                            autonPlan = 1;
	                        }
	                        else { //Starting at left, switch is left, scale is right
	                            autonPlan = 2;
	                        }
	                    } else {
	                        autonPlan = 3; //Starting at left, switch is right, scale is left
	                    }
	                } else {
	                    autonPlan = 4; //Starting at left, switch and scale are right
	                }
	            } else if (startPos == 2){
	                if (switchLeft) {
	                    if (scaleLeft) autonPlan = 5; //Starting at middle, go to switch at left then scale at left
	                    else autonPlan = 6; //Starting at middle, go to switch at left, then scale at right
	                } else {
	                    if (scaleLeft) autonPlan = 7; //Starting at middle, go to switch at right then scale at left
	                    else autonPlan = 8; //Starting at middle, go to switch at right then scale at right
	                }
	            } else if (startPos == 3){
	                if (!switchLeft || !scaleLeft){
	                    if (!switchLeft){
	                        if (!scaleLeft) { //Starting at right, switch and scale are right
	                            autonPlan = 9;
	                        }
	                        else { //Starting at right, switch is right, scale is left
	                            autonPlan = 10;
	                        }
	                    } else {
	                        autonPlan = 11; //Starting at right, switch is left, scale is right
	                    }
	                } else {
	                    autonPlan = 12; //Starting at right, switch and scale are left
	                }
	            }
	        }
	    }


	    void AutonomousPeriodic() {
	    //Dimesions -> https://firstfrc.blob.core.windows.net/frc2018/Drawings/LayoutandMarkingDiagram.pdf
	            //Playable Field Width -> (29.69*2 + 264) inches
	            //Playable Field Length -> (288*2 + 72) inches
	            if (autoSelected == autoNameCustom) {
	                // Custom Auto goes here
	            } else {
	                // Default Auto goes here
	                bool complete = false;
	                double right = 90.0;
	                double left = -90.0;
	                double width = (29.69*2 + 264);
	                double length = (288*2 + 72);
	                double switchWidth = 196.00 - 140.00;
	                double switchLength = width - (85.25*2);
	                double theGap = 261.47 - 196.00;
	                double gapToSwitch = theGap/2 + (switchWidth/2);
	                double gapToScale = theGap/2 + (length/2 - 261.47);
	                if (pid->distOrAngle == 1) complete = pid->drive_distance();
	                else if (pid->distOrAngle == 2) complete = pid->turn_angle();
	                if (complete || autonStage == 0){
	                    //All of these cases need a few statements at the beginning about getting from the starting point to the first destination
	                    //Each robot starting place is 6 feet wide
	                    switch (autonPlan){
	                        case 1: //Starting at left, switch and scale are left
	                            switch (autonStage) {
	                                case 0: //Drive to the switch
	                                    pid->set_goal(140.00 + switchWidth/2);
	                                    break;
	                                case 1: //Finished driving forward, turn towards switch
	                                    pid->set_angle(right);
	                                    break;
	                                case 2: //At switch, now place cube
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 3: //Turn back, facing scale
	                                    pid->set_angle(left);
	                                    break;
	                                case 4: //Drive between switch and scale
	                                    pid->set_goal(gapToSwitch);
	                                    break;
	                                case 5: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 6: //Drive forward to cubes
	                                    pid->set_goal(width/2 - 85.25/2);
	                                    break;
	                                case 7: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 8: //Pick up cube
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 9: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 10: //Drive parallel to scale
	                                    pid->set_goal(width/2 - 71.57/2);
	                                    break;
	                                case 11: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 12: //Drive close to scale
	                                    pid->set_goal(gapToScale);
	                                    break;
	                                case 13: //Turn right towards the scale
	                                    pid->set_angle(right);
	                                    break;
	                                case 14: //Place cube on scale
	                                    pid->reset();
	                                    //???
	                                    break;
	                            }
	                            break;

	                        case 2: //Starting at left, switch is left, scale is right
	                            switch (autonStage) {
	                                case 0: //Drive 168 inches to the switch
	                                    pid->set_goal(168.0);
	                                    break;
	                                case 1: //Arrived at switch, turn right towards it
	                                    pid->set_angle(right);
	                                    break;
	                                case 2: //Now place cube
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 3: //Turn back to face scale
	                                    pid->set_angle(left);
	                                    break;
	                                case 4: //Drive straight to between scale and switch
	                                    pid->set_goal(gapToSwitch);
	                                    break;
	                                case 5: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 6: //Drive to cubes
	                                    pid->set_goal(width/2 - 85.25/2);
	                                    break;
	                                case 7: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 8: //Pick up cube
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 9: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 10: //Drive parallel to scale
	                                    pid->set_goal(width/2 - 71.57/2);
	                                    break;
	                                case 11: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 12: //Drive closer to scale
	                                	pid->set_goal(gapToScale);
	                                	break;
									case 13: //Turn left towards scale
										pid->set_angle(left);
										break;
									case 14: //Place cube on scale
										pid->reset();
										//???
										break;
	                            }
	                            break;

	                        case 3: //Starting at left, switch is right, scale is left
	                            //Are we doing scale first in this case?
	                            switch (autonStage) {
	                                case 0: //Drive to left scale
	                                    pid->set_goal(324.0); //Check this
	                                    break;
	                                case 1: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 2: //Place cube on scale
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 3: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 4: //Drive to between scale and switch
	                                    pid->set_goal(95.265);
	                                    break;
	                                case 5: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 6: //Drive towards cube
	                                    pid->set_goal(85.25/2 + (width/2 - 85.25/2));
	                                    break;
	                                case 7: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 8: //Pick up cube
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 9: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 10: //Drive parallel to the switch to between it and the scale
	                                    pid->set_goal(width/2 - 85.25/2);
	                                    break;
	                                case 11: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 12: //Drive to switch
	                                    pid->set_goal((261.47-196)/2 - (196-140)/2);
	                                    break;
	                                case 13: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 14: //Place cube on switch
	                                    pid->reset();
	                                    //???
	                                    break;
	                            }
	                            break;

	                        case 4: //Starting at left, switch and scale are right
	                            switch (autonStage) {
	                                case 0: //Drive to between the switch and scale
	                                    pid->set_goal(196 + (261.47-196)/2);
	                                    //pid->set_goal(228.735);
	                                    break;
	                                case 1: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 2: //Drive parallel to the switch to between it and the scale
	                                    pid->set_goal(85.25/2 + (width - 85.25*2) + 85.25/2);
	                                    break;
	                                case 3: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 4: //Drive to switch
	                                    pid->set_goal((261.47-196)/2 + (196-140)/2);
	                                    break;
	                                case 5: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 6: //Place cube on switch
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 7: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 8: //Drive to in between switch and scale
	                                    pid->set_goal((261.47-196)/2 + (196-140)/2);
	                                    break;
	                                case 9: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 10: //Drive to cube, parallel to switch
	                                    pid->set_goal(85.25/2 + (width/2 - 85.25*2));
	                                    break;
	                                case 11: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 12: //Pick up cube
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 13: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 14: //Drive parallel to the scale
	                                    pid->set_goal(85.25/2 + (width - 71.57));
	                                    break;
	                                case 15: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 16: //Drive to scale
	                                    pid->set_goal((261.47-196)/2 + 72/2);
	                                    break;
	                                case 17: //Turn left left towards scale
	                                    pid->set_angle(left);
	                                    break;
	                                case 18: //Place cube on scale
	                                    pid->reset();
	                                    //???
	                                    break;

	                            }
	                            break;

	                        case 5: //Starting at middle, go to switch at left, then scale at left
	                            switch (autonStage) {
	                                case 0: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 1: //Drive past switch
	                                    pid->set_goal();
	                                    break;
	                                case 2: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 3: //Drive to switch
	                                    pid->set_goal();
	                                    break;
	                                case 4: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 5: //Place cube on switch
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 6: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 7: //Drive past switch
	                                    pid->set_goal((196-140)/2 + (261.47-196)/2);
	                                    break;
	                                case 8: //Turn right
										  pid->set_angle(right);
										  break;
	                                case 9: //Drive forward to cube
	                                	  pid->set_goal(width/2 - 85.25/2);
	                                      break;
	                                case 10: //Turn right
	                                      pid->set_angle(right);
	                                      break;
	                                case 11: //Pick up cube
	                                       pid->reset();
	                                     //???
	                                    break;
	                                case 12: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 13: //Drive parallel to scale
	                                    pid->set_goal(width/2 - 71.57/2);
	                                    break;
	                                case 14: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 15: //Drive to scale
	                                    pid->set_goal((261.47-196)/2 + (288 - 261.47) + 72/2);
	                                    break;
	                                case 16: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 17: //Place cube on scale
	                                    pid->reset();
	                                    //???
	                                    break;
	                            }
	                            break;

	                        case 6: //Starting at middle, go to switch at left, then scale at right
	                            switch (autonStage) {
	                                case 0: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 1: //Drive past switch
	                                    pid->set_goal();
	                                    break;
	                                case 2: //Turn right
	                                	pid->set_angle(right);
	                                	break;
									case 3: //Drive to switch
										pid->set_goal();
										break;
									case 4: //Turn right
										pid->set_angle(right);
										break;
									case 5: //Place cube on switch
										pid->reset();
										//???
										break;
									case 6: //Turn left
										pid->set_angle(left);
										break;
									case 7: //Drive between switch and scale
										pid->set_goal((196-140)/2 + (261.47-196)/2);
										break;
									case 8: //Turn right
										pid->set_angle(right);
										break;
									case 9: //Drive up to cube
										pid->set_goal(width/2 - 85.25/2);
										break;
									case 10: // Turn right
										pid->set_angle(right);
										break;
									case 11: //Pick up cube
									pid->reset();
									break;
									case 12: //Turn left
									pid->set_angle(left);
									break;
									case 13: //Drive parallel to scale
									pid->set_goal(width/2 - 71.57/2);
									break;
									case 14: //Turn left
									pid->set_angle(left);
									break;
									case 15: //Drive to scale
									pid->set_goal((261.47-196)/2 + (288 - 261.47) + 72/2);
									break;
									case 16: //Turn left
									pid->set_angle(left);
									break;
									case 17: //Place cube on scale
									pid->reset();
									//???
									break;
	                           }
	                            break;

	                        case 7: //Starting at middle, go to switch at right, then scale at left
	                            switch (autonStage) {
	                                case 0: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 1: //Drive past switch
	                                    pid->set_goal();
	                                    break;
	                                case 2: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 3: //Drive to switch
	                                    pid->set_goal();
	                                    break;
	                                case 4: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 5: //Place cube on switch
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 6: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 7: //Drive to middle of scale and switch
	                                    pid->set_goal((196-140)/2 + (261.47-196)/2);
	                                    break;
	                                case 8: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 9: //Drive forward to cube
										pid->set_goal(width/2 - 85.25/2);
										break;
									case 10: //Turn left
										pid->set_angle(left);
										break;
									case 11: //Pick up cube
										pid->reset();
										//???
	                            		break;
									case 12: //Turn right
										pid->set_angle(right);
										break;
									case 13: //Drive parallel to scale
										pid->set_goal(width/2 - 71.57/2);
										break;
									case 14: //Turn right
										pid->set_angle(right);
										break;
									case 15: //Drive to scale
										pid->set_goal((261.47-196)/2 + (288 - 261.47) + 72/2);
										break;
									case 16: //Turn right
										pid->set_angle(right);
										break;
									case 17: //Place cube on scale
										pid->reset();
										/???
										break;
	                            }
	                            break;

	                        case 8: //Starting at middle, go to switch at right, then scale at right
	                            switch (autonStage) {
	                                case 0: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 1: //Drive past switch
	                                    pid->set_goal();
	                                    break;
	                                case 2: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 3: //Drive to switch
	                                    pid->set_goal();
	                                    break;
	                                case 4: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                                case 5: //Place cube on switch
	                                    pid->reset();
	                                    //???
	                                    break;
	                                case 6: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                                case 7: //Drive to between scale and switch
	                                    pid->set_goal((196-140)/2 + (261.47-196)/2);
	                                    break;
	                                case 8: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                               case 9: //Drive forward to cube
	                                    pid->set_goal(width/2 - 85.25/2);
	                                    break;
	                               case 10: //Turn left
										pid->set_angle(left);
										break;
	                               case 11: //Pick up cube
										pid->reset();
										//???
										break;
	                               case 12: //Turn left
										pid->set_angle(left);
										break;
	                             case 13: //Drive parallel to scale
										pid->set_goal(width/2 - 71.57/2);
										break;
	                               case 14: //Turn left
										pid->set_angle(left);
										break;
	                               case 15: //Drive to scale
										pid->set_goal((261.47-196)/2 + (288 - 261.47) + 72/2);
										break;
	                               case 16: //Turn left
										pid->set_angle(left);
										break;
	                               case 17: //Place cube on scale
										pid->reset();
										//???
										break;
	                            }
	                            break;

	                        case 9: //Starting at right, switch and scale are right
	                            switch (autonStage) {
	                               case 0: //Drive 168 inches to the switch
										pid->set_goal(168.0);
										break;
	                               case 1: //Turn left
										pid->set_angle(left);
										break;
	                               case 2: //Place cube
										pid->reset();
										//???
										break;
	                               case 3: //Turn right
										pid->set_angle(right);
										break;
	                               case 4: //Drive between switch and scale
										pid->set_goal((196-140)/2 + (261.47-196)/2);
										break;
	                               case 5: //Turn left
										pid->set_angle(left);
										break;
	                               case 6: //Drive forward to cube
										pid->set_goal(width/2 - 85.25/2);
										break;
	                               case 7: //Turn left
										pid->set_angle(left);
										break;
	                               case 8: //Pick up cube
										pid->reset();
										//???
										break;
	                               case 9: //Turn left
										pid->set_angle(left);
										break;
	                               case 10: //Drive parallel to scale
										pid->set_goal(width/2 - 71.57/2);
										break;
	                               case 11: //Turn left
										pid->set_angle(left);
										break;
	                               case 12: //Drive to scale
										pid->set_goal((261.47-196)/2 + (288 - 261.47) + 72/2);
										break;
	                               case 13: //Turn left
										pid->set_angle(left);
										break;
	                               case 14: //Place cube on scale
										pid->reset();
										//???
										break;
	                            }
	                            break;

	                        case 10: //Starting at right, switch is right, scale is left
	                            switch (autonStage) {
	                               case 0: //Drive to switch
	                                    pid->set_goal(168.0);
	                                    break;
	                               case 1: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                               case 2: //Place cube
	                                    pid->reset();
	                                    //???
	                                    break;
	                               case 3: //Turn right
	                                    pid->set_angle(right);
	                                    break;
	                               case 4: //Drive to between scale and switch
	                                    pid->set_goal((196-140)/2 + (261.47-196)/2);
	                                    break;
	                               case 5: //Turn left
	                                    pid->set_angle(left);
	                                    break;
	                               case 6: //Drive forward to cube
										pid->set_goal(width/2 - 85.25/2);
										break;
	                               case 7: //Turn left
										pid->set_angle(left)
										break;
	                               case 8: //Pick up cube
										pid->reset();
										//???
										break;
	                               case 9: //Turn right
										pid->set_angle(right);
										break;
	                               case 10: //Drive parallel to scale
										pid->set_goal(width/2 - 71.57/2);
										break;
	                               case 11: //Turn right
										pid->set_angle(right);
										break;
	                               case 12: //Drive to scale
										pid->set_goal((261.47-196)/2 + (288 - 261.47) + 72/2);
										break;
	                               case 13: //Turn right
										pid->set_angle(right);
										break;
	                               case 14: //Place cube on scale
										pid->reset();
										//???
										break;
	                            }
	                            break;

	                        case 11: //Starting at right, switch is left, scale is right
	                            switch (autonStage) {
	                                case 0: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 1: //Drive past switch
	                                   pid->set_goal();
	                                   break;
	                                case 2: //Turn right
	                                   pid->set_angle(right);
	                                   //???
	                                   break;
	                                case 3: //Drive to switch
	                                   pid->set_goal();
	                                   break;
	                                case 4: //Turn right
	                                   pid->set_angle(right);
	                                   break;
	                                case 5: //Place cube
	                                   pid->reset();
	                                   //????
	                                   break;
	                                case 6: //Drive to switch
	                                   pid->set_goal();
	                                   break;
	                                case 7: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 8: //Pick up cube
	                                   pid->reset();
	                                   //???
	                                   break;
	                                case 9: //Turn right
	                                   pid->set_angle(right);
	                                   break;
	                                case 10: //Drive to past the switch
	                                   pid->set_goal();
	                                   break;
	                                case 11: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 12: //Drive to switch
	                                   pid->set_goal();
	                                   break;
	                                case 13: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 14: //Place cube on switch
	                                   pid->reset();
	                                   //???
	                                   break;
	                            }
	                            break;

	                        case 12: //Starting at right, switch and scale are left
	                            switch (autonStage) {
	                                case 0: //Drive to between the switch and scale
	                                   pid->set_goal(228.735);
	                                   break;
	                                case 1: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 2: //Drive past switch
	                                   pid->set_goal();
	                                   break;
	                                case 3: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 4: //Drive to switch
	                                   pid->set_goal();
	                                   break;
	                                case 5: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 6: //Place cube on switch
	                                   pid->reset();
	                                   //???
	                                   break;
	                                case 7: //Turn left
	                                   pid->set_angle(left);
	                                   break;
	                                case 8: //Drive to scale
	                                   pid->set_goal();
	                                   break;
	                                case 9: //Turn right
	                                   pid->set_angle(right);
	                                   break;
	                                case 10: //Place cube on scale
	                                   pid->reset();
	                                   //???
	                                   break;
	                            }
	                            break;
	                    }
	                    autonStage++;
	                }

	            }
	        }

	void TeleopInit() {}

	void TeleopPeriodic() {
		driving->move(joy1->GetRawAxis(1), joy2->GetRawAxis(1));

	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
