/*
 * rplidar41.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: RoboWarriors
 */

#include <rplidar41.h>
#define RESULT_OK              0

rplidar_41::rplidar_41(int baudrate, SerialPort::Port port) {
	// TODO Auto-generated constructor stub
	serial = new SerialPort(baudrate,port);
}

//u_result rplidar_41::send_command(command cmda)
u_result rplidar_41::send_command(_u8 cmd, const void * payload, size_t payloadsize)
{
	//
	//
	/*const char * buff;
	buff += command::RPLIDAR_ANS_SYNC_BYTE1;



	serial->Write();*/

	_u8 pkt_header[10];
	rplidar_cmd_packet_t * header = reinterpret_cast<rplidar_cmd_packet_t * >(pkt_header);
	_u8 checksum = 0;

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
			checksum ^= ((_u8 *)payload)[pos];
		}

		// send size
		_u8 sizebyte = payloadsize;
		//_rxtx->senddata(&sizebyte, 1);
		serial->Write((const char*)&sizebyte, 1);

		// send payload
		//_rxtx->senddata((const _u8 *)payload, sizebyte);
		serial->Write((const char*)payload, sizebyte);
		// send checksum
		//_rxtx->senddata(&checksum, 1);
		serial->Write((const char*)checksum, 1);
	}

	return RESULT_OK;
}


rplidar_41::~rplidar_41() {
	// TODO Auto-generated destructor stub
}

