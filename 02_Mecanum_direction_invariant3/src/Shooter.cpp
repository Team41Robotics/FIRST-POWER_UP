#include <Shooter.h>
#include <WPILib.h>
#define MaxVolt 1
#define MaxAngle 1


Shooter::Shooter ()
{
	timer = new Timer();
	timer->Reset();
	timer->Start();
	iCount = 0;

	dOld = 0;

	angPerVolt = MaxAngle/MaxVolt;
}

void Shooter::get_delta_time ()
{
	curr_t = timer->Get();    //unsure what units. need it in seconds.
	del_t = curr_t - old_t;
	old_t = curr_t;
}

void Shooter::aim_shooter(Encoder *encode, AnalogPotentiometer *pot, CANTalon *M_shooter)
{
	double err = getTargetAngle(0,0,0) - getAngle(pot);
	iCount += err;
	double speed = K_sp * err + K_si*iCount + K_sd*(err - dOld);
	dOld = err;
	M_shooter->Set(speed);
}

double Shooter::minD(double a,double b)
{
	printf("a=%f\n",a*(180.0/PI));
	printf("b=%f\n",b*(180.0/PI));
	if(isnan(a) && isnan(b))
		return 1/0;
	if(isnan(a))
		return b;
	if(isnan(b))
		return a;
	if(a<b)
		return a;
	else
		return b;
}

double Shooter::getTargetAngle(double v,double y, double x) //UPDATE FOR DRAG. This is in inches.
{
	return 180.0 / PI * minD ( atan( (v*v + sqrt(v*v*v*v - G*(G*x*x + 2*y*v*v) ) )/(G*x) ) , atan((v*v - sqrt(v*v*v*v - G*(G*x*x + 2*y*v*v)))/(G*x)) );
	//return 180.0 / PI * minD( atan( v*v + sqrt( v*v*v*v - G * (G * x*x + 2 * y * v*v) )  / (G * x) ),atan( v*v - sqrt( v*v*v*v - G * (G * x*x + 2 * y * v*v) ) / (G * x) ) );
}









double Shooter::getVelocity(double E_del_ang) //also Coleman’s formula
{
	return (E_del_ang * RADIUS)/del_t;
}

double Shooter::getAngle(AnalogPotentiometer *pot)
{
	return pot->Get() * angPerVolt;
}
