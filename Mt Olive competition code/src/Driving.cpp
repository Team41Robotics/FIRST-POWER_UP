/*
 * tankdrivetest.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#include <Driving.h>

Driving::Driving() {
	leftTalon0 = new TalonSRX(TALON::LEFT_FRONT);
	leftTalon1 = new TalonSRX(TALON::LEFT_BACK);
	rightTalon0 = new TalonSRX(TALON::RIGHT_FRONT);
	rightTalon1 = new TalonSRX(TALON::RIGHT_BACK);

	rt_encoder_talon = rightTalon0;
	lf_encoder_talon = leftTalon0;
	wharhs = new  AHRS(SerialPort::Port::kUSB1);

	resetAuto();

//	turbo = false;
//	turbo_toggle = false;
	//leftTalon0 = new CANTalon(0);//these IDs have to be changed occasionally using the roborio-41-frc.local thing
	//rightTalon0 = new CANTalon(2);
	//leftTalon1 = new CANTalon(3);
	//rightTalon1 = new CANTalon(1);
}

void Driving::drive(double leftspeed, double rightspeed)
{
	leftTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftspeed);
	rightTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightspeed);
	leftTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftspeed);
	rightTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightspeed);
}

double signum(double a)
{
	if(a < 0)
		return -1.0;
	return 1.0;
}

double clamp(double val, double low, double high)
{
	if(val > high)
		return high;
	if(val <low)
		return low;
	return val;
}

double magnitude (double a, double b)
{
	return sqrt(a*a+b*b);
}

void Driving::resetPosition()
{
	pos.reset();
}

double wrap(double a, double bounds)
{
	if(a < 0)
		return bounds + a;
	if(a >= bounds)
		return a - bounds;
	return a;
}

void Driving::updatePosition()
{




	/*left_n = lf_encoder_talon->GetSensorCollection().GetQuadraturePosition()*(PI/180.0)*WHEEL_RADIUS/1024.0;		//say what it do
	right_n = rt_encoder_talon->GetSensorCollection().GetQuadraturePosition()*(PI/180.0)*WHEEL_RADIUS/1024.0;
	left_d = left_n - left_o;
	right_d = right_n - right_o;
	left_o = left_n;
	right_o = right_n;
	if (fabs(left_d - right_d) < 1.0e-6) {
		pos.x += left_d * cos(pos.theta);
		pos.y += right_d * sin(pos.theta);
	    //pos.theta = pos.theta;
	} else {
	    float R = AXLE_LENGHT * (left_d + right_d) / (2 * (right_d - left_d));
	    float wd = (right_d - left_d) / AXLE_LENGHT;

	    pos.x += R * sin(wd + pos.theta) - R * sin(pos.theta);
	    pos.y -= R * cos(wd + pos.theta) + R * cos(pos.theta);
	    pos.theta = wrap(pos.theta + wd, 2 * PI); // forces it to be [0,2PI)
	}
*/
	left_encoder = lf_encoder_talon->GetSensorCollection().GetQuadraturePosition();
	right_encoder = rt_encoder_talon->GetSensorCollection().GetQuadraturePosition();

	pos.x = (-lf_encoder_talon->GetSensorCollection().GetQuadraturePosition() / 4096.0) * ( PI * WHEEL_RADIUS *2);// TODO: DOMINQUE NEEDS TO BE NEGATIVE HERE I BELIEVE
	pos.y = (rt_encoder_talon->GetSensorCollection().GetQuadraturePosition() / 4096.0) * ( PI * WHEEL_RADIUS * 2);


	///23.5 5.75
}

/*void Driving::tankDrive(Joystick *joyLeft,Joystick *joyRight) {

	if(joyLeft->GetRawButtonPressed(1))		//toggles between turbo and normal speed modes
	{
		if(!turbo_toggle)
		{
			turbo = !turbo;
		}
		turbo_toggle = 1;
	}
	else
	{
		turbo_toggle = 0;
	}


	//throttle = -1.0*clamp(joy1->GetRawAxis(1),-1.0,0.0)*(max_throt - min_throt) + min_throt;




	move(joyLeft->GetRawAxis(1), joyRight->GetRawAxis(1));

	//move(joyRight->GetRawAxis(1) - joyRight->GetRawAxis(0) , joyRight->GetRawAxis(1) + joyRight->GetRawAxis(0));


	if(turbo)
	{
		//	move(joy1->GetRawAxis(1)*0.6, joy2->GetRawAxis(1)*0.6);
	}
	else
	{
		//	move(joy1->GetRawAxis(1)*0.6, joy2->GetRawAxis(1)*0.6);
		//move(joy1->GetRawAxis(1)/4, joy2->GetRawAxis(1)/4);
		//move(joy1->GetRawAxis(1)/4, joy1->GetRawAxis(5)/4);
	}

}
*/

void Driving::tankDrive(Joystick *joyLeft,Joystick *joyRight, Joystick *buttonBoard)
{
	if(buttonBoard != NULL){
		//stuff with button board control...
		//maybe speed.
	}

	//throttle = -1.0*clamp(joy1->GetRawAxis(1),-1.0,0.0)*(max_throt - min_throt) + min_throt;

	SmartDashboard::PutNumber("RIGHT ENCODER VALUE",rt_encoder_talon->GetSensorCollection().GetQuadraturePosition());
	SmartDashboard::PutNumber("LEFT ENCODER VALUE",lf_encoder_talon->GetSensorCollection().GetQuadraturePosition());

	SmartDashboard::PutNumber("leftspeed",joyLeft->GetRawAxis(1));
	SmartDashboard::PutNumber("rightspeed",joyRight->GetRawAxis(1));

//	SmartDashboard::PutNumber("left Joystick",joyLeft->GetRawAxis(1));
//	SmartDashboard::PutNumber("right Joystick",joyRight->GetRawAxis(1));


	drive(joyLeft->GetRawAxis(1), joyRight->GetRawAxis(1)/*-0.09448818862438202*/);////the constant is a fudge factor to get it to stop going backwards.
}

void Driving::ControllerMove(Joystick * controller)
{
	drive(controller->GetRawAxis(1)*0.4 - controller->GetRawAxis(0)*0.4, controller->GetRawAxis(1)*0.4 + controller->GetRawAxis(0)*0.4);
}

void Driving::setEncoders()
{
	right_encoder = rt_encoder_talon->GetSelectedSensorPosition(0);// rightTalon0->GetSelectedSensorPosition(0);
	left_encoder = lf_encoder_talon->GetSelectedSensorPosition(0);//leftTalon0->GetSelectedSensorPosition(0);
	leftTalon0->GetSensorCollection().GetQuadraturePosition(); // this is also a way to access the encoder.
}

void Driving::resetEncoders()
{
	rt_encoder_talon->GetSensorCollection().SetQuadraturePosition(0,500);
	lf_encoder_talon->GetSensorCollection().SetQuadraturePosition(0,500);
//	rightTalon0->GetSelectedSensorPosition(0);
//	leftTalon0->GetSelectedSensorPosition(0);
//	leftTalon0->GetSensorCollection().GetQuadraturePosition(); // this is also a way to access the encoder.
}

/*bool Driving::AutoForwards(double goal)
{
	int polarity = -signum(goal);
	goal = fabs(goal);
	double k_p = 1.0/50;
	current = ((fabs(pos.x)+fabs(pos.y))*1.04)/2.0;
	//printf("current %f\n", current);
	error = goal - current;
	speed = error*k_p;
	integral += speed*k_i;
	if(integral > 0.4)
		integral = 0.4;
	//	drive(speed * 0.4, speed * 0.4);

	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;


	if(fabs(error) < TOLERANCE)
	{
		printf("error: %f\n",error);
		drive(0.0, 0.0);
		return true;
	}
	else if (speed > 1.0){
		drive(0.5, 0.5);
	}
	else
	{
		drive(polarity*(speed*0.5+(integral)), polarity*(speed*0.5+(integral)));
	}

	return false;
}*/

/*bool Driving::AutoTurn(double angle){

	current_angle= wharhs->GetAngle();
	error_angle = fabs(angle - current_angle);
//	int polarity = signnum (error_angle);
	if(error_angle < 0.5){
		drive(0.0,0.0);
		return true;
	}
	error_angle /= 90.0;
	integral_angle +=  error_angle;
	drive(((k_pAngle*-error_angle)+(k_iAngle*-integral_angle)+(-(previous_angle-current_angle)*k_dAngle))*signum(angle),((k_pAngle*error_angle)+(k_iAngle*integral_angle)+(-(previous_angle-current_angle)*k_dAngle))*signum(angle));
	previous_angle = current_angle;

	return false;
}*/
//----- WITH OSCILLATION------//


bool Driving::AutoTurn(double angle, double tolerance){
	if (firstTime) {
		resetAuto();
		firstTime = false;
	}
	current_angle= wharhs->GetAngle();
	error_angle = angle - current_angle;
//	int polarity = signnum (error_angle);
	if(fabs(error_angle) < tolerance){
		drive(0.0,0.0);
		firstTime = true;
		return true;
	}
	error_angle /= 90.0;
	integral_angle +=  error_angle;
	if(integral_angle > 45)// cap integral
		integral_angle = 45;
	else if(integral_angle < -45)
		integral_angle = -45;
	double speed = ((k_pAngle*-error_angle)+(k_iAngle*-integral_angle)+(-(previous_angle-current_angle)*k_dAngle));
	if (speed > 1)
		speed = 1;
	else if (speed < -1)
		speed = -1;

	printf("speed: %f\n",speed);
	drive(speed,-speed);
	previous_angle = current_angle;

	return false;
}

bool Driving::AutoForwards(double goal)
{
	if (firstTime) {
		resetAuto();
		firstTime = false;
	}
	current = (((pos.x+pos.y))*1.04)/2.0;
	//printf("current %f\n", current);
	error = goal - current;
	speed = -error*k_p;
	integral += speed*k_i;
	if(integral > 0.2)
		integral = 0.2;
	if(integral < -0.2)
		integral = -0.2;

	//	drive(speed * 0.4, speed * 0.4);

	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;


	if(fabs(current) > fabs(goal))//fabs(error) < TOLERANCE)
	{
		printf("error: %f\n",error);
		drive(0.0, 0.0);
		firstTime = true;
		return true;
	}
	else if (speed > 1.0){
		drive(0.75, 0.75);
	}
	else if (speed < -1.0){
		drive(-0.75, -0.75);
	}
	else
	{
		drive((speed*0.4+(integral)), (speed*0.4+(integral)));
	}

	return false;
}



///PRECODITION:
// rplidar object is made.
// a call to
// lidar->begin(SerialPort::Port::kUSB1);
// lidar->lidar_thread();
// lidar->startScan(true,500);
// has been made.
// it will crash if not. an uncaught nll_ptr.
// IN MILIMETERS
bool Driving::LidarForwards(double goal, RPLidar * lidar)
{
	if (firstTime) {
		resetAuto();
		firstTime = false;
	}
	current = lidar->getRangeDistance(5,355,true);
	//printf("current %f\n", current);
	error = goal - current;
	speed = error*k_p;
	integral += speed*k_i;
	if(integral > 0.2)
		integral = 0.2;
	if(integral < -0.2)
		integral = -0.2;

	//	drive(speed * 0.4, speed * 0.4);

	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;


	if(fabs(current) > fabs(goal))//fabs(error) < TOLERANCE)
	{
		printf("error: %f\n",error);
		drive(0.0, 0.0);
		firstTime = true;
		return true;
	}
	else if (speed > 1.0){
		drive(0.75, 0.75);
	}
	else if (speed < -1.0){
		drive(-0.75, -0.75);
	}
	else
	{
		drive((speed*0.4+(integral)), (speed*0.4+(integral)));
	}

	return false;
}


void Driving::resetAuto(){
	resetEncoders();
	resetPosition();
	wharhs->Reset();
	integral = 0;
	integral_angle = 0;
	error = 0;
	error_angle = 0;
}

bool Driving::autoCube(Lift * lift){
	if (firstTime) {
		resetAuto();
		firstTime = false;
	}
	//k_pCube = SmartDashboard::GetNumber("K P cube",0.0);
	//k_iCube = SmartDashboard::GetNumber("K I cube",0.0);
	//k_dCube = SmartDashboard::GetNumber("K D cube",0.0);


	current = -SmartDashboard::GetNumber("cmxn2", 0.0);//may need to invert.
	if(fabs(current) < 0.07){ //Tolerance is 2.0. IS this an okay value? isn't it on [-1,1]?
		drive(0.0,0.0);
		firstTime = true;
		lift->runClaw(0.0);
		return true;
	}
	lift->runClaw(0.0);
	integral +=  current;
	double iMax = 0.3;
	if(integral_angle > iMax)// cap integral
		integral_angle = iMax;
	else if(integral_angle < -iMax)
		integral_angle = -iMax;
	double speed = ((k_pCube*-current)+(k_iCube*-integral)+(-(previous-current)*k_dCube));
	/*if (speed > 0.5)
		speed = 0.5;
	else if (speed < -0.5)
		speed = -0.5;*/

	double creep_bias = 0.0;
	printf("oskar: %f\n",speed);

	drive(speed+creep_bias,-(speed+creep_bias));
	previous = current;

	return false;
}
