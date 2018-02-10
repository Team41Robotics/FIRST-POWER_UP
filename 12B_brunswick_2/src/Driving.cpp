#include <WPILib.h>
#include <WPILib_auxiliary.h>
#include <Driving.h>
#include <math.h>

Driving::Driving ()
{
	M_fr = new CANTalon(9);//9
	M_fl = new CANTalon(3);//3
	M_br = new CANTalon(6);//6
	M_bl = new CANTalon(7);//7
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

void Driving::Mecanum_drive(Joystick *control_joy){

	double forward = -1*(control_joy -> GetY());
	double right = control_joy ->GetX();
	double clockwise = control_joy -> GetZ();

	double front_left = forward + clockwise + right;
	double front_right = forward - clockwise - right;
	double rear_left = forward + clockwise - right;
	double rear_right = forward - clockwise + right;

	double max = abs((int)front_left);

	if(abs((int)front_right)>max) max = abs((int)front_right);
	if(abs((int)rear_left)>max) max = abs((int)rear_left);
	if(abs((int)rear_right)>max) max = abs((int)rear_right);

	if(max>1){
		front_left = front_left / max;
		front_right /=max;
		rear_left /=max;
		rear_right /=max;
	}

	Move(front_right, front_left, rear_right, rear_left);

}
void Driving::Manual_driving (Joystick *control_joy)//,ADXRS450_Gyro *gyro)//This doesn't work
{
	con_x = control_joy->GetRawAxis(0);
    con_y = control_joy->GetRawAxis(1);
    con_t = control_joy->GetRawAxis(2);

    scl = absD(con_x) + absD(con_y) + absD(con_t);
//    if(scl == 0)
//   	throtTemp = 0.0;
//    printf("scl: %f\t",scl);
    scl = max( scl , 1.0 );

    if(control_joy->GetPOV() != -1.0)
        direction_angle = control_joy->GetPOV();

    throt = ((-control_joy->GetRawAxis(3)+1)/2.0)*throt_rng + throt_min;

    if(throt > 0.7)
    {
    	if(throtTemp <= throt && throtTemp <= 1.0)
    		throtTemp += 0.01;
    	else if(throtTemp > throt)
    		throtTemp = throt;
        throt = throtTemp;
    }
    else
    	throtTemp = 0.7;


    switch( direction_angle )                 //conventional
	{
		case 0 :
			Move(
			(con_y + con_x + con_t) / scl ,
			(con_y - con_x - con_t) / scl ,
			(con_y - con_x + con_t) / scl ,
			(con_y + con_x - con_t) / scl
			);
			break;
		case 90 : //might not work. I just assumed the axes changes. whatever
			Move(
			( -con_y + con_x + con_t) / scl ,
			( +con_y + con_x - con_t) / scl ,
			( +con_y + con_x + con_t) / scl ,
			( -con_y + con_x - con_t) / scl
			);
			break;
		case 180 :
			Move(
			(-con_y - con_x + con_t) / scl ,
			(-con_y + con_x - con_t) / scl ,
			(-con_y + con_x + con_t) / scl ,
			(-con_y - con_x - con_t) / scl
			);
			break;
		case 270 :
			Move(
			( + con_y -con_x + con_t) / scl ,
			( - con_y -con_x - con_t) / scl ,
			( - con_y -con_x + con_t) / scl ,
			( + con_y -con_x - con_t) / scl
			);
			break;
		case -1 :
			//printf(“CHOOSE DIRECTION\n”);
			break;
	}

}

void Driving::Move ( double fr , double fl , double br , double bl )
{
    M_fr->Set( fr * throt );
    M_fl->Set( -fl * throt );
    M_br->Set( br * throt );
    M_bl->Set( -bl * throt );
}

void Driving::Auto_Move ( double fr , double fl , double br , double bl )
{
    M_fr->Set( fr );
    M_fl->Set( -fl );
    M_br->Set( br  );
    M_bl->Set( -bl );
}

void Driving::Auto_Stop ()
{
	M_fr->Set( 0.0 );
	M_fl->Set( 0.0 );
	M_br->Set( 0.0 );
	M_bl->Set( 0.0 );
}

void Driving::Auto_Foward (double speed){
	Auto_Move(speed,speed,speed,speed);
}

int Driving::LidarDist(){
	lidar->reset();
	int distance;
	if(lidar->isMeasurementValid(false)){
		distance = lidar->getDistance();
		SmartDashboard::PutNumber("lidar",distance/2.54);
		SmartDashboard::PutBoolean("IN RANGE?",((distance/2.54) > 12.0 && (distance/2.54) < 14.0));
		printf("dist %f\n",distance/2.54);
		return distance;
	}
	else {
		printf("INVALID DISTANCE\n");
		return -1;
	}
}

bool Driving::NOPID_Move (float Dist,float Speed, bool Foward)
{
//	imu->Localization(nav);
	float lidDist = LidarDist()/2.54;
	//int moveDistance = lidDist*0.9 + imu->position_y*0.1;
	//SmartDashboard::PutNumber("smaller lazer-y prateek. ",lidDist);
	int moveDistance = lidDist;
	if(moveDistance < Dist && Speed > 0  && Foward){
		Auto_Foward(Speed);
		return false;
	}
	else if(moveDistance > Dist && Speed < 0  && !Foward){
		Auto_Foward(Speed);
		return false;
	}
	else{
		Auto_Stop();
		return true;
	}
}

bool Driving::NOPID_Move_Dist (float Dist,float Speed)
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

bool Driving::NOPID_Turn (float angle,float Speed)
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
		Auto_Move( Speed * negativeTurn , -Speed * negativeTurn , Speed * negativeTurn, -Speed * negativeTurn);
		return false;
	}
	else{
		Auto_Stop();
		return true;
	}
}

bool Driving::Pid_Move_upTo (double distance)
{
	double drive_curr = lidar->getDistance();

	if(drive_curr >= distance)
	{
		drive_error_accum = 0;
		drive_error_old = 0;
		return true;
	}
	else
	{
		drive_error = drive_curr-distance;

		drive_out = K_drive_p * drive_error + K_drive_i * drive_error_accum + K_drive_d * (drive_error - drive_error_old);
		drive_error_accum += drive_error;

		if(drive_error_accum > MAX_DRIVE_ACCUM)
			drive_error_accum = MAX_DRIVE_ACCUM;
		if(drive_error_accum < -MAX_DRIVE_ACCUM)
			drive_error_accum = -MAX_DRIVE_ACCUM;
		drive_error_old = drive_error;

		Auto_Move( drive_out , drive_out , drive_out , drive_out );
		return false;
	}
}

bool Driving::Pid_Move_downTo (double distance)
{
	double drive_curr = lidar->getDistance();
	if(drive_curr <= distance)
	{
		drive_error_accum = 0;
		drive_error_old = 0;
		return true;
	}
	else
	{
		drive_error = drive_curr-distance;

		drive_out = K_drive_p * drive_error + K_drive_i * drive_error_accum + K_drive_d * (drive_error - drive_error_old);
		drive_error_accum += drive_error;

		if(drive_error_accum > MAX_DRIVE_ACCUM)
			drive_error_accum = MAX_DRIVE_ACCUM;
		if(drive_error_accum < -MAX_DRIVE_ACCUM)
			drive_error_accum = -MAX_DRIVE_ACCUM;
		drive_error_old = drive_error;

		Auto_Move( -drive_out , -drive_out , -drive_out , -drive_out );
		return false;
	}
}

bool Driving::Pid_turn_downTo (double angle)
{
	if(imu->theta <= angle)
	{
		turn_error_accum = 0;
		turn_error_old = 0;
		return true;
	}
	else
	{
		turn_error = angle-imu->theta;

		turn_out = K_turn_p * turn_error + K_turn_i * turn_error_accum + K_turn_d * (turn_error - turn_error_old);
		turn_error_accum += turn_error;

		if(turn_error_accum > MAX_TURN_ACCUM)
			turn_error_accum = MAX_TURN_ACCUM;
		if(turn_error_accum < -MAX_TURN_ACCUM)
			turn_error_accum = -MAX_TURN_ACCUM;

		//printf("e: %f\ti = %f\n", turn_error, turn_error_accum);
		turn_error_old = turn_error;

		Move( turn_out , -turn_out , turn_out , -turn_out );  //should turn. should.

		return false;
	}
}

bool Driving::Pid_turn_upTo (double angle)
{
	if(imu->theta >= angle)
	{
		turn_error_accum = 0;
		turn_error_old = 0;
		return true;
	}
	else
	{
		turn_error = angle-imu->theta;

		turn_out = K_turn_p * turn_error + K_turn_i * turn_error_accum + K_turn_d * (turn_error - turn_error_old);
		turn_error_accum += turn_error;

		if(turn_error_accum > MAX_TURN_ACCUM)
			turn_error_accum = MAX_TURN_ACCUM;
		if(turn_error_accum < -MAX_TURN_ACCUM)
			turn_error_accum = -MAX_TURN_ACCUM;

		//printf("e: %f\ti = %f\n", turn_error, turn_error_accum);
		turn_error_old = turn_error;

		Move( -turn_out , turn_out , -turn_out , turn_out );  //should turn. should.

		return false;
	}
}
