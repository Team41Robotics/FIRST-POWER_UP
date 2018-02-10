/*
 * rplidar41.h
 *
 *  Created on: Jan 27, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_RPLIDAR41_H_
#define SRC_RPLIDAR41_H_
#include <SerialPort.h>
#include <bitset>
#include <Timer.h>
#define NULL ((void*)0)
#define RESULT_OK              0

typedef struct rp_point
{
	double distance;
	double angle;
};


class rplidar_41 {
public:
	enum command
	{
		RPLIDAR_CMD_STOP				=0x25,
		RPLIDAR_CMD_SCAN           		=0x20,
		RPLIDAR_CMD_FORCE_SCAN     		=0x21,
		RPLIDAR_CMD_RESET            	=0x40,
		RPLIDAR_CMD_GET_DEVICE_INFO  	=0x50,
		RPLIDAR_CMD_GET_DEVICE_HEALTH 	=0x52,
		RPLIDAR_CMD_GET_SAMPLERATE 		=0x59,
		RPLIDAR_CMD_EXPRESS_SCAN    	=0x82,
		RPLIDAR_CMD_SET_MOTOR_PWM    	=0xF0,
		RPLIDAR_CMD_GET_ACC_BOARD_FLAG 	=0xFF,
		RPLIDAR_EXPRESS_SCAN_MODE_NORMAL=0,
		RPLIDAR_EXPRESS_SCAN_MODE_FIXANGLE  =  1,
		MAX_MOTOR_PWM             =  1023,
		DEFAULT_MOTOR_PWM         =  660,
		RPLIDAR_ANS_TYPE_DEVINFO         = 0x4,
		RPLIDAR_ANS_TYPE_DEVHEALTH       = 0x6,
		RPLIDAR_ANS_TYPE_MEASUREMENT          =      0x81,
		RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED =      0x82,
		RPLIDAR_ANS_TYPE_SAMPLE_RATE     = 0x15,
		RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG   =0xFF,
		RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK     = (0x1),
		RPLIDAR_STATUS_OK						=0x0,
		RPLIDAR_STATUS_WARNING           		=0x1,
		RPLIDAR_STATUS_ERROR             		= 0x2,
		RPLIDAR_RESP_MEASUREMENT_SYNCBIT       = (0x1<<0),
		RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT = 2,
		RPLIDAR_RESP_MEASUREMENT_CHECKBIT      = (0x1<<0),
		RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT   = 1,
		RPLIDAR_CMD_SYNC_BYTE        =0xA5,
		RPLIDAR_CMDFLAG_HAS_PAYLOAD  =0x80,
		RPLIDAR_ANS_SYNC_BYTE1      = 0xA5,
		RPLIDAR_ANS_SYNC_BYTE2    =   0x5A,
		RPLIDAR_ANS_PKTFLAG_LOOP =   0x1,
		RPLIDAR_ANS_HEADER_SIZE_MASK  =      0x3FFFFFFF,
		RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT  =  (30)
	};


	int send_command (uint8_t cmd, const void * payload, size_t payloadsize);
	rplidar_41(int, SerialPort::Port);
	virtual ~rplidar_41();
	void send_command ();
	void stop();
	void start();
	void get_measurement();
	rp_point current;
private:
	SerialPort *serial;
};


/*
typedef struct _rplidar_cmd_packet_t {
    uint8_t syncByte; //must be RPLIDAR_CMD_SYNC_BYTE
    uint8_t cmd_flag;
    uint8_t size;
    uint8_t data[0];
} __attribute__((packed)) rplidar_cmd_packet_t;
*/

#endif /* SRC_RPLIDAR41_H_ */
