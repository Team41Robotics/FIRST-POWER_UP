#include <WPILib.h>
#include <WPILib_auxiliary.h>
#include <Driving.h>
#include <math.h>


Driving::Driving ()
{
    M_fr = new CANTalon(1);
    M_fl = new CANTalon(2);
    M_br = new CANTalon(3);
    M_bl = new CANTalon(4);
    throt = 0.0;
    throt_min = .13;
    throt_rng = 0.87;
    direction_angle = 0;
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

    printf("e: %f\ti = %f\n", aim_error, aim_error_accum);
    aim_error_old = aim_error;

    Move( aim_out , -aim_out , aim_out , -aim_out );  //should turn. should.
}

void Driving::Mecanum_drive(Joystick *control_joy, ADXRS450_Gyro *gyro){

	double forward = -1*(control_joy -> GetY());
	double right = control_joy ->GetX();
	double clockwise = control_joy -> GetZ();



	double temp = forward*cos(gyro->GetAngle()) - right*sin(gyro->GetAngle());
	right = forward*sin(theta) + right*cos(theta);
	forward = temp;

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
void Driving::Manual_driving (Joystick *control_joy,ADXRS450_Gyro *gyro)//This doesn't work
{
    con_x = control_joy->GetRawAxis(0);
    con_y = control_joy->GetRawAxis(1);
    con_t = control_joy->GetRawAxis(2);

    scl = absD(con_x) + absD(con_y) + absD(con_t);
    scl = max( scl , 1.0 );

    if(control_joy->GetPOV() != -1.0)
        direction_angle = control_joy->GetPOV();

    theta = gyro->GetAngle()*RAD;
    SmartDashboard::PutNumber("rotation",theta/RAD);

    throt = ((-control_joy->GetRawAxis(3)+1)/2.0)*throt_rng + throt_min;

	if(control_joy->GetRawButton(2)) //for direction invariant translation
	{
		SmartDashboard::PutBoolean("orientation invariance",true);

		//scale factor on each controller value - changes whether + or    USSR -
		double fix_theta = -theta;
		double cms = cos(fix_theta) - sin(fix_theta); // 0: 1 	90: -1 	180: -1 	270: 1
		double cps = cos(fix_theta) + sin(fix_theta); // 0: 1 	90: 1 	180: -1 	270: -1
		double smc = sin(fix_theta) - cos(fix_theta); // 0: -1 	90: 1 	180: 1 	270: -1
		//double out = cms + cps;

		scl = absD(con_x) + absD(con_y) + absD(con_t);
		scl = max( scl , 1.0 );

		if(theta*1/RAD>=180)
		{
			Move(
				(con_y*(cms) + con_x*(cps) + con_t) / scl,
				(con_y*(cps) - con_x*(cms) - con_t) / scl,
				(con_y*(cps) + con_x*(cms) + con_t) / scl,
				(con_y*(cms) - con_x*(cps) - con_t) / scl
			);
		}
		else
		{
			Move(
					(con_y*(cms) - con_x*(cps) + con_t) / scl,
					(con_y*(cps) + con_x*(cms) - con_t) / scl,
					(con_y*(cps) - con_x*(cms) + con_t) / scl,
					(con_y*(cms) + con_x*(cps) - con_t) / scl
				);
		}
		printf("theta %f\n", theta*1/RAD);
		printf("cps %f\n",cps);
		printf("cms %f\n",cms);

//printf("as %f\n",sin(theta));

	} else {
		SmartDashboard::PutNumber("direction",direction_angle);
		SmartDashboard::PutBoolean("orientation invariance",false);
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
}

void Driving::Move ( double fr , double fl , double br , double bl )
{
    M_fr->Set( fr * throt );
    M_fl->Set( -fl * throt );
    M_br->Set( br * throt );
   printf("fr %f ",fr);
   printf("fl %f ",fl);
   printf("br %f ",br);
   printf("bl %f \n",bl);

    M_bl->Set( -bl * throt );
}

void Driving::Auto_Move ( double fr , double fl , double br , double bl )
{
    M_fr->Set( fr );
    M_fl->Set( -fl );
    M_br->Set( br  );
   printf("fr %f ",fr);
   printf("fl %f ",fl);
   printf("br %f ",br);
   printf("bl %f \n",bl);

    M_bl->Set( -bl );
}
