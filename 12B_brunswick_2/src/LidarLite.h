
#pragma once

#include "I2C.h"
#include <wpilib.h>
#include <atomic>
#include <memory>
#include <set>
#include <string>
#include <thread>
#define LIDARADDRESS 0X62

class LidarLite: frc::I2C
{

public:

	//Register list
	//command name, address, read and/or write, description, initial value if any
	enum Commands {
		kACQ_COMMAND 	  = 0x00, //W Device command
		kSTATUS 		  = 0x01, //R System status
		kSIG_COUNT_VAL	  = 0x02, //R/W Maximum acquisition count 0x80
		kACQ_CONFIG_REG   = 0x04, //R/W  Acquisition mode control 0x08
		kVELOCITY 		  = 0x09, //R Velocity measurement output
		kPEAK_CORR 		  = 0x0c, //R Peak value in correlation record
		kNOISE_PEAK 	  = 0x0d, //R  Correlation record noise floor
		kSIGNAL_STRENGTH  = 0x0e, //R Received signal strength
		kFULL_DELAY_HIGH  = 0x0f, //R Distance measurement high byte
		kFULL_DELAY_LOW   = 0x10, //R Distance measurement low byte
		kOUTER_LOOP_COUNT = 0x11, //R/W Burst measurement count control 0x01
		kREF_COUNT_VAL 	  = 0x12, //R/W Reference acquisition count 0x05
		kLAST_DELAY_HIGH  = 0x14, //R Previous distance measurement high byte
		kLAST_DELAY_LOW   = 0x15, //R Previous distance measurement low byte
		kUNIT_ID_HIGH 	  = 0x16, //R Serial number high byte Unique
		kUNIT_ID_LOW 	  = 0x17, //R Serial number low byte Unique
		kI2C_ID_HIGH 	  = 0x18, //W Write serial number high byte for I2C address unlock
		kI2C_ID_LOW		  = 0x19, //W Write serial number low byte for I2C address unlock
		kI2C_SEC_ADDR 	  = 0x1a, //R/W Write new I2C address after unlock
		kTHRESHOLD_BYPASS = 0x1c, //R/W Peak detection threshold bypass 0x00
		kI2C_CONFIG 	  = 0x1e, //R/W Default address response control 0x00
		kCOMMAND 		  = 0x40, //R/W State command
		kMEASURE_DELAY 	  = 0x45, //R/W Delay between automatic measurements  0x14
		kPEAK_BCK 		  = 0x4c, //R Second largest peak value in correlation record
		kCORR_DATA		  = 0x52, //R Correlation record data low byte
		kCORR_DATA_SIGN   = 0x53, //R Correlation record data high byte
		kACQ_SETTINGS 	  = 0x5d, //R/W Correlation record memory bank select
		kPOWER_CONTROL 	  = 0x65, //R/W Power state control 0x80
		kDIST_HIGHLOW 	  = 0x8F  //R register to read high and low byte of distance
	};
	enum DistanceUnit { kInches = 0, kMilliMeters = 1 };
	LidarLite(I2C::Port port, int deviceaddress);
	LidarLite(I2C::Port port);
	~LidarLite();
	void SetFreeRun(bool enable);
	void reset();
	uint16_t getDistance();
	double getVelocity();
	void configure(int configuration);
	void setUpdateDelay(uint8_t delay);
	bool isBusy();
	uint8_t getConfig();
	uint8_t getStatus();
	bool isMeasurementValid(bool healthy);

private:
	const int defaultaddress = 0x62;
	bool m_enabled = false;
	bool m_freerun = false;
	bool m_biascorrectionenable = false;
	int measuredelay = 0x14;
	void getMeasurement();
	void setConfig(uint8_t config);
	void initialize();
	void measurementRepeat(uint8_t count);

};


