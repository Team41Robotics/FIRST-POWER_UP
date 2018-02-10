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

double Shooter::getTargetAngle(double v,double y, double x) //UPDATE FOR DRAG
{
	return 180.0 / PI * atan( (v*v + sqrt( v*v*v*v - G * (G * x*x + 2 * y * v*v) ) ) / (G * x) );
}

double Shooter::getVelocity(double E_del_ang) //also Coleman’s formula
{
	return (E_del_ang * RADIUS)/del_t;
}

double Shooter::getAngle(AnalogPotentiometer *pot)

{
	return pot->Get() * angPerVolt;
}
