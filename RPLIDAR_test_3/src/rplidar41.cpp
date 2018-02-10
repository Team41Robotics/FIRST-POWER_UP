/*
 * rplidar41.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: RoboWarriors
 */
/*
#include "rplidar41.h"
#include <SerialPort.h>
#include "rplidar_protocol.h"

rplidar_41::rplidar_41(int baudrate, SerialPort::Port port) {
	// TODO Auto-generated constructor stub
	serial = new SerialPort(baudrate,port);
}

//u_result rplidar_41::send_command(command cmda)
int rplidar_41::send_command(uint8_t cmd, const void * payload, size_t payloadsize)
{
	//
	//
	/*const char * buff;
	buff += command::RPLIDAR_ANS_SYNC_BYTE1;



	serial->Write();* /

	uint8_t pkt_header[10];
	rplidar_cmd_packet_t * header = reinterpret_cast<rplidar_cmd_packet_t * >(pkt_header);
	uint8_t checksum = 0;

	if (payloadsize && payload) {
		cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
	}

	header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
	header->cmd_flag = cmd;

	// send header first
	//_rxtx->senddata(pkt_header, 2) ;
	serial->Write((const char*)pkt_header, 2);	//untested.


	if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
		checksum ^= RPLIDAR_CMD_SYNC_BYTE;
		checksum ^= cmd;
		checksum ^= (payloadsize & 0xFF);

		// calc checksum
		for (size_t pos = 0; pos < payloadsize; ++pos) {
			checksum ^= ((uint8_t *)payload)[pos];
		}

		// send size
		uint8_t sizebyte = payloadsize;
		//_rxtx->senddata(&sizebyte, 1);
		serial->Write((const char*)&sizebyte, 1);

		// send payload
		//_rxtx->senddata((const uint8_t *)payload, sizebyte);
		serial->Write((const char*)payload, sizebyte);
		// send checksum
		//_rxtx->senddata(&checksum, 1);
		serial->Write((const char*)checksum, 1);
	}

	return RESULT_OK;
}


void rplidar_41::stop()
{
	send_command(command::RPLIDAR_CMD_STOP, NULL, 0);
}

void rplidar_41::start()
{
	//send_command(command::RPLIDAR_CMD_SCAN, NULL, 0);S

	/*char * buf = new char[2];//0xa5;
	buf[0] = 165;
	buf[1] = 0x20;
*/
/*
	std::string buf = "00";
	buf[0] = 0xA5;
	buf[1] = 0x25;
	//serial->Flush();
	serial->Write(buf);
	Wait(0.01);
* /
	char * ch;
	ch += 0xA5;
	ch += 0x21;
	/*
	buf = "00";
	buf[0] = 0xA5;
	buf[1] = 0x21;
	//serial->Flush();
	* ///serial->Reset();
	serial->Write(ch,2);
}

void rplidar_41::get_measurement ()
{
	char * buff = new char [7];

	printf("bytes received %i\n",serial->GetBytesReceived());

	serial->Read(buff, 7);


	char flag0 = buff[0];
	char flag1 = buff[1];
	if(flag0 != 0xA5 || flag1 != 0x5A)
	{
		return;
	}

	char c0 = buff[2];
	char c1 = buff[3];
	char c2 = buff[4];
	char c3 = buff[5];
	char c4 = buff[6];

	printf("0:%c,1:%c,2:%c,3:%c,4:%c,5:%c,1:%c,6:%c\n",flag0,flag1,c0,c1,c2,c3,c4  );


	std::bitset<8> b0(c0);
	std::bitset<8> b1(c1);
	std::bitset<8> b2(c2);
	std::bitset<8> b3(c3);
	std::bitset<8> b4(c4);


	bool S = b0[0];
	//if S is 1, its a new scan line
	bool Sn = b0[1];

	//the rest is pointless for byte one

	std::bitset<15> angle_q6;
	angle_q6[0] = b1[1];
	angle_q6[1] = b1[2];
	angle_q6[2] = b1[3];
	angle_q6[3] = b1[4];
	angle_q6[4] = b1[5];
	angle_q6[5] = b1[6];
	angle_q6[6] = b1[7];

	angle_q6[7] = b2[0];
	angle_q6[8] = b2[1];
	angle_q6[9] = b2[2];
	angle_q6[10] = b2[3];
	angle_q6[11] = b2[4];
	angle_q6[12] = b2[5];
	angle_q6[13] = b2[6];
	angle_q6[14] = b2[7];

	long int angle_int = 0;

	for(int i=0;i<15;i++)
	{
		angle_int += angle_q6[i] * pow(2,i);
	}
	double angle = ((double)angle_int)/64.0;
	//angle should be proper. Assuming it passes the least significant first... else we flip it

	std::bitset<16> distance_q2;
	distance_q2[0] = b3[0];
	distance_q2[1] = b3[1];
	distance_q2[2] = b3[2];
	distance_q2[3] = b3[3];
	distance_q2[4] = b3[4];
	distance_q2[5] = b3[5];
	distance_q2[6] = b3[6];
	distance_q2[7] = b3[7];

	distance_q2[8] = b4[0];
	distance_q2[9] = b4[1];
	distance_q2[10] = b4[2];
	distance_q2[11] = b4[3];
	distance_q2[12] = b4[4];
	distance_q2[13] = b4[5];
	distance_q2[14] = b4[6];
	distance_q2[15] = b4[7];

	long int distance_int = 0;

	for(int i=0;i<16;i++)
	{
		distance_int += distance_q2[i] * pow(2,i);
	}
	double distance = ((double)distance_int)/4.0;

	rp_point out;
	out.angle = angle;
	out.distance = distance;
	current = out;


	//first bit is S
	//second bit is !S
	//next six are quality.

	//first is C
	//next 7 are angle
	//next 8 are angle
	//next 8 are distance
	//next 8 are distance
}

rplidar_41::~rplidar_41() {
	// TODO Auto-generated destructor stub
}
*/
