#include <WPILib.h>
#include <Driving.h>
#include <math.h>

Driving::Driving ()
{
	M_fr = new CANTalon(1);//9
	M_fl = new CANTalon(2);//3
	M_br = new CANTalon(3);//6
	M_bl = new CANTalon(4);//7
    throt = 0.0;
    throt_min = .13;
    throt_rng = 0.87;
    direction_angle = 0;
	//nav = new AHRS(I2C::Port::kOnboard);
    nav = new AHRS(SerialPort::Port::kUSB1);
    nav->Reset();
	imu = new IMU();
	imu->Reset(nav);
	lidar = new LidarLite(I2C::Port::kOnboard,0X62);
}

double Driving::absD (double a) //this may need to be put in a new class. I assumed “Aux_functions”. Only if it is used in other classes.
{
    if(a < 0)
        return -a;
    else
        return a;
}

double Driving::max(double a,double b)
{
	if(a > b)
		return a;
	else
		return b;
}

void Driving::Auto_aim ( double cmxn )
{
    aim_error = -cmxn;

    aim_out = K_aim_p * aim_error + K_aim_i * aim_error_accum + K_aim_d * (aim_error - aim_error_old);
    aim_error_accum += aim_error;

    if(aim_error_accum > MAX_AIM_ACCUM)
        aim_error_accum = MAX_AIM_ACCUM;
    if(aim_error_accum < -MAX_AIM_ACCUM)
        aim_error_accum = -MAX_AIM_ACCUM;

    ///printf("e: %f\ti = %f\n", aim_error, aim_error_accum);
    aim_error_old = aim_error;

    Move( aim_out , -aim_out , aim_out , -aim_out );  //should turn. should.
}

void Driving::Manual_driving (Joystick *control_joy)//,ADXRS450_Gyro *gyro)//This doesn't work
{
	con_x = control_joy->GetRawAxis(0);
    con_y = control_joy->GetRawAxis(1);
    con_t = control_joy->GetRawAxis(2);

    scl = absD(con_x) + absD(con_y) + absD(con_t);
    scl = max( scl , 1.0 );

    if(control_joy->GetPOV() != -1.0)
        direction_angle = control_joy->GetPOV();


    throt = ((-control_joy->GetRawAxis(3)+1)/2.0)*throt_rng + throt_min;

	switch( direction_angle )                 //conventional
	{
		case 0 :
			Move(
			((con_y + con_x + con_t) / scl) * throt,
			((con_y - con_x - con_t) / scl) * throt,
			((con_y - con_x + con_t) / scl) * throt,
			((con_y + con_x - con_t) / scl) * throt
			);
			break;
		case 90 : //might not work. I just assumed the axes changes. whatever
			Move(
			(( -con_y + con_x + con_t) / scl) * throt,
			(( +con_y + con_x - con_t) / scl) * throt,
			(( +con_y + con_x + con_t) / scl) * throt,
			(( -con_y + con_x - con_t) / scl) * throt
			);
			break;
		case 180 :
			Move(
			((-con_y - con_x + con_t) / scl) * throt ,
			((-con_y + con_x - con_t) / scl) * throt,
			((-con_y + con_x + con_t) / scl) * throt,
			((-con_y - con_x - con_t) / scl) * throt
			);
			break;
		case 270 :
			Move(
			(( + con_y -con_x + con_t) / scl) * throt,
			(( - con_y -con_x - con_t) / scl) * throt,
			(( - con_y -con_x + con_t) / scl) * throt,
			(( + con_y -con_x - con_t) / scl) * throt
			);
			break;
		case -1 :
			//printf(“CHOOSE DIRECTION\n”);
			break;
	}

}

void Driving::Move ( double fr , double fl , double br , double bl )
{
    M_fr->Set( fr );
    M_fl->Set( -fl );
    M_br->Set( br );
    M_bl->Set( -bl );
}

void Driving::Auto_Foward (double speed){
	Move(speed,speed,speed,speed);
}

void Driving::Auto_Stop ()
{
	Auto_Foward(0.0);
}

int Driving::LidarDist(){
	lidar->reset();
	int distance;
	if(lidar->isMeasurementValid(false)){
		distance = lidar->getDistance();
		//SmartDashboard::PutNumber("lidar",distance);
		return distance;
	}
	else {
		printf("INVALID DISTANCE");
		return -1;
	}
}

bool Driving::Auto_Move (float Dist,float Speed, bool Foward)
{
//	imu->Localization(nav);
	float lidDist = LidarDist()/2.54;
	//int moveDistance = lidDist*0.9 + imu->position_y*0.1;
	SmartDashboard::PutNumber("smaller lazer-y prateek. ",lidDist);
	int moveDistance = lidDist;
	if(moveDistance < Dist && Foward){
		Auto_Foward(Speed);
		return false;
	}
	else if(moveDistance > Dist && !Foward){
		Auto_Foward(Speed);
		return false;
	}
	else{
		Auto_Stop();
		return true;
	}
}

bool Driving::Auto_Move_Dist (float Dist,float Speed)
{
	float lidDist = LidarDist()/2.54;

	if(firstDist == -1)
	{
		firstDist = lidDist;
	}

	SmartDashboard::PutNumber("smaller lazer-y prateek. ",lidDist);
	if(lidDist < firstDist + Dist && Dist > 0 ){
		Auto_Foward(Speed);
		return false;
	}
	else if(lidDist > firstDist + Dist && Dist < 0 ){
		Auto_Foward(Speed);
		return false;
	}
	else{
		Auto_Stop();
		firstDist = -1;
		return true;
	}
}

bool Driving::Auto_Turn (float angle,float Speed)
{
	int negativeTurn = 1;
	float curr_angle = imu->theta;
	if(angle < 0){
		curr_angle*=-1;
		angle*=-1;
		negativeTurn = -1;
	}
	printf("curangle: %f\n",curr_angle);
	if(curr_angle < angle) {
		Move( Speed * negativeTurn , -Speed * negativeTurn , Speed * negativeTurn, -Speed * negativeTurn);
		return false;
	}
	else{
		Auto_Stop();
		return true;
	}
}

bool Driving::Auton_Gear(bool rightTurn){
	int polarity = rightTurn ? 1: -1;
	if(auton_step == 0)
	{
		if(Auto_Move(114.5,0.3))
			auton_step = 1;
	}
	else if(auton_step == 1)
	{
		if(Auto_Turn(-120.0 * polarity,0.3))
			auton_step = 2;
	}
	else if(auton_step == 2)
	{
		if(Auto_Move(20.0,-0.3,false))
			auton_step = 3;
	}
	else
	{
		Auto_Stop();
		return true;
	}
	return false;
}
