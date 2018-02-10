#include <WPILib.h>
#include <WPILib_auxiliary.h>
#include <Driving.h>



Driving::Driving ()
{
    M_fr = new CANTalon(9);
    M_fl = new CANTalon(3);
    M_br = new CANTalon(6);
    M_bl = new CANTalon(7);
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

    aim_error_old = aim_error;

    Move( aim_out , -aim_out , aim_out , -aim_out );  //should turn. should.
}

void Driving::Manual_driving (Joystick *control_joy)
{
    con_x = control_joy->GetRawAxis(0);
    con_y = control_joy->GetRawAxis(1);
    con_t = control_joy->GetRawAxis(2);

    scl = absD(con_x) + absD(con_y) + absD(con_t);
    scl = max( scl , 1.0 );

    if(control_joy->GetPOV() != -1.0)
        direction_angle = control_joy->GetPOV();

    SmartDashboard::PutNumber("rotation",theta/RAD);

    throt = ((-control_joy->GetRawAxis(3)+1)/2.0)*throt_rng + throt_min;

	if(control_joy->GetRawButton(2)) //for direction invariant translation
	{
		SmartDashboard::PutBoolean("orientation invariance",true);
		///*
		/* Move(                         //UNDER DEVELOPMENT
		( cos(theta) + sin(theta) ) * (con_y - con_x + con_t) / scl ,
		( cos(theta) + sin(theta) ) * (con_y + con_x - con_t) / scl ,
		( cos(theta) + sin(theta) ) * (con_y - con_x + con_t) / scl ,
		( cos(theta) + sin(theta) ) * (con_y + con_x - con_t) / scl
		);*/
		angle_factor_pp = cos(theta) + sin(theta);
		angle_factor_pm = cos(theta) - sin(theta);
		angle_factor_mp =-cos(theta) + sin(theta);
		printf("pp: %f\n",angle_factor_pp);
		printf("pm: %f\n",angle_factor_pm);
		printf("mp: %f\n",angle_factor_mp);
		Move(
		( angle_factor_pm * con_y + angle_factor_pp * con_x + con_t) / scl ,
		( angle_factor_pp * con_y - angle_factor_mp * con_x - con_t) / scl ,
		( angle_factor_pp * con_y - angle_factor_mp * con_x + con_t) / scl ,
		( angle_factor_pm * con_y + angle_factor_pp * con_x - con_t) / scl
		);
		/*


	(cos(theta) - sin(theta))
	(cos(theta) + sin(theta))
	+
		(con_y + con_x + con_t) / scl , 	0
		(-con_y + con_x + con_t) / scl ,		90
		(-con_y - con_x + con_t) / scl ,	180
		(con_y - con_x + con_t) / scl		270

	(cos(theta) + sin(theta))
	(-cos(theta) + sin(theta))
	-
		(con_y - con_x - con_t) / scl ,
		(con_y + con_x - con_t) / scl ,
		(-con_y + con_x - con_t) / scl ,
		(-con_y - con_x - con_t) / scl

	(cos(theta) + sin(theta))
	(-cos(theta) + sin(theta))
	+
		(con_y - con_x + con_t) / scl ,
		(con_y + con_x + con_t) / scl ,
		(-con_y + con_x + con_t) / scl ,
		(-con_y - con_x + con_t) / scl

	(cos(theta) - sin(theta))
	(cos(theta) + sin(theta))
	-
		(con_y + con_x - con_t) / scl
		(-con_y + con_x - con_t) / scl
		(-con_y - con_x - con_t) / scl
		(con_y - con_x - con_t) / scl



		 */
		//*/
	} else {
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
			(con_x - con_y + con_t) / scl ,
			(con_x + con_y - con_t) / scl ,
			(con_x + con_y + con_t) / scl ,
			(con_x - con_y - con_t) / scl
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
			(-con_x + con_y + con_t) / scl ,
			(-con_x - con_y - con_t) / scl ,
			(-con_x - con_y + con_t) / scl ,
			(-con_x + con_y - con_t) / scl
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
   // printf(“thx, david. %f\n”,throt);
    M_bl->Set( -bl * throt );
}
