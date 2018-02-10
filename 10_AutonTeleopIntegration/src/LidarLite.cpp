#include "LidarLite.h"


	LidarLite::LidarLite(I2C::Port port, int deviceaddress)
		:I2C(port, deviceaddress)
	{

		m_biascorrectionenable = true;
		m_freerun = true;//FREE RUN ON
		initialize();
	}

	LidarLite::LidarLite(I2C::Port port)
		:I2C(port, defaultaddress)
	{
		LidarLite(port, defaultaddress);
	}
	LidarLite::~LidarLite()
	{

	}

	void LidarLite::SetFreeRun(bool enable)
	{
		if (enable)
			measurementRepeat(0xff);
//		else
//			measurementRepeat(0x01);
	}

	void LidarLite::setUpdateDelay(uint8_t delay)
	/*
	 * 0xff = 255 = 7.8hz
	 * 0xc8 = 200 = 10 hz
	 * 0x14 = 20 = 100 hz
	 * 0x01 = 2000hz
	 * 0x00 = ?????
	 *
	 * delay is in units of 0.5ms
	 */
	{
		//set our delay given
		Write(Commands::kMEASURE_DELAY,delay);

		measuredelay = delay;
		uint8_t config = getConfig();

		//set bit 5
		config|= 0b100000;

		setConfig(config);

	}

	void LidarLite::initialize()
	{
		SetFreeRun(m_freerun);
	}


	void LidarLite::reset()
	//resets the device
	{

		//send reset
		Write(Commands::kACQ_COMMAND, 0x00);

//		initialize();
	}

	void LidarLite::getMeasurement()
	//resets the device
	{
	    Wait(.02);
		Write(Commands::kACQ_COMMAND, 0x04);

	}


	uint16_t LidarLite::getDistance()
	//returns in cm
	{
		  int16_t distance = 0;
		  if(!m_freerun)
			  getMeasurement();
		  else
			  Wait(0.05);

		  unsigned char distanceRegister[1];
		  unsigned char LidarData[2];
//		  while(isBusy());
		  distanceRegister[0] = kDIST_HIGHLOW;
		  WriteBulk(distanceRegister,1);
		  ReadOnly(2,LidarData);
		  distance = (LidarData[0]<<8)+LidarData[1];
		  return distance;
	}

	double LidarLite::getVelocity()
	/* doesnt work, fix it
	 * returns in cm/s
	 * assumes we are in freerun as this class does not clock itself
	 * if delay is 0 it will crash
	 */
	{
		double distance = 0;
		double velocity = 1;

		//returns signed 2's comp from the last measurement to current
		Read(Commands::kVELOCITY,1,reinterpret_cast<uint8_t*>(&distance));
		//distance is in 2's complement

		if (distance >128)
		{
			//2's complement it
			distance = ~((uint32_t)distance)+1;
			velocity = -1;
		}

		if (measuredelay != 0)
			velocity = (velocity*distance)/(double)measuredelay;
		else
			velocity = (velocity*distance);

		return velocity;
	}

	/***
	 *
	 * Private
	 *
	 */


	void LidarLite::measurementRepeat(uint8_t count)
	{
		Write(Commands::kOUTER_LOOP_COUNT,count);
	}

	uint8_t LidarLite::getStatus()
	{
		uint8_t status = 0;
		unsigned char statusRegister[1];

		statusRegister[0] = kSTATUS;
		WriteBulk(statusRegister,1);
	    ReadOnly(1,&status);
		return status;
	}

	void LidarLite::setConfig(uint8_t config)
	{
		Write(Commands::kACQ_CONFIG_REG,config);
	}

	uint8_t LidarLite::getConfig()
	{
		uint8_t config = 0;

		unsigned char configRegister[1];

		configRegister[0] = kACQ_CONFIG_REG;
		WriteBulk(configRegister,1);
		ReadOnly(1,(&config));
		return config;
	}
	bool LidarLite::isMeasurementValid(bool healthy)
	{
		/**bit 6 set processor error
		 * bit 5 clear health error
		 * bit 4 set secondary return should not make measurement invalid
		 * bit 3 set no return peak detected
		 * bit 2 dont care
		 * bit 1 dont care
		 * bit 0 dont care
		 *
		 *
		 * bit number
		 * 6543210
		 * 01X0XXX mask of a good measurement
		 * 0100000 AND it with 1101000 do clear bits we dont care about
		 * If we get anything other than 010000 our measurement sucks
		 */
		uint8_t status = getStatus();
		//printf("stat %d\t",status);
		//printf("true? %d\n",(status & 0b1101000) == 0b0100000);
		if(healthy)
			return ((status & 0b1101000) == 0b0100000);
		else
			return ((status & 0b1001000) == 0b0000000);

	}
	bool LidarLite::isBusy()
	{
		return (getStatus() & 0x01) == 0x01;
	}
