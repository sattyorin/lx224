#include "lx224_controllers/lx224.h"

lx224::lx224()
	:socket_fd_(-1),
	baudrate_(B115200),
	packet_start_time_(0.0),
	packet_timeout_(0.0),
	tx_time_per_byte(0.0)
{}

lx224::~lx224()
{
	closePort();
}


bool lx224::SERVO_MOVE_TIME_WRITE(uint8_t id, int16_t pos, uint16_t t) //short packet
{
	uint8_t packet[10];

	memset(packet, 0, sizeof(packet)); //clear buffer

	packet[PKT_HEADER1] = 0x55;
	packet[PKT_HEADER2] = 0x55;
	packet[PKT_ID] = id; //0x00-0xFD, 0xFE is the broadcast ID
	packet[PKT_LENGTH] = 7;
	packet[PKT_COMMAND] = 1;

	if(pos < 0)
		pos = 0;
	if(pos > 1000)
		pos = 1000;

	packet[PKT_PARAMETER1] = (pos & 0xFF); //lower 8bit of the angle value
	packet[PKT_PARAMETER1+1] = ((pos >> 8) & 0xFF); //higher 8bit of the angle value
	packet[PKT_PARAMETER1+2] = (t & 0xFF); //lower 8bit of the time value
	packet[PKT_PARAMETER1+3] = ((t >> 8) & 0xFF); //higher 8bit of the time value
	packet[PKT_PARAMETER1+4] = checkSum(packet); //chechsum
	flushPort(); //
	if ( (packet[PKT_LENGTH] + 3) != writePort(packet, packet[PKT_LENGTH] + 3))
	{
		return false;
	}
	else
	{
		return true;
	}
}

uint16_t lx224::SERVO_POS_READ(uint8_t id)
{
	uint8_t packet[6];
	uint8_t rpacket[6];

	memset(packet, 0, sizeof(packet)); //clear buffer
	memset(packet, 0, sizeof(rpacket));

	packet[PKT_HEADER1] = 0x55;
	packet[PKT_HEADER2] = 0x55;
	packet[PKT_ID] = id;
	packet[PKT_LENGTH] = 3;
	packet[PKT_COMMAND] = 20;

	packet[PKT_PARAMETER1+4] = checkSum(packet);
	flushPort(); 
	if (packet[PKT_LENGTH] + 3 !=  writePort(packet, packet[PKT_LENGTH] + 3))
	{
		printf("[lx224::SERVO_POS_READ] Checksum of received packet does not match!\n");
		return -999;
	}
	else
	{
		if (readPort(rpacket, 6) == 6)
		{
			if(rpacket[PKT_HEADER1] == 0x55 && rpacket[PKT_HEADER2] == 0x55)
			{
				return (rpacket[PKT_PARAMETER1] & 0xFF) | ((rpacket[PKT_PARAMETER1+1] & 0xFF) << 8);
			}
			else
			{
				printf("[lx224::SERVO_POS_READ] Received packet has wrong header!\n");
				return -997;
			}
		}
		else
		{
			printf("[lx224::SERVO_POS_READ] Length of received packet does not match!\n");
			return -998;
		}
	}
}

uint8_t lx224::checkSum(uint8_t pkt[])
{
	uint8_t temp = 0;
	for (uint8_t i = 2; i < pkt[PKT_LENGTH] + 2; i++)
		temp += pkt[i];
	temp = ~temp;
	return temp;
}

// bool lx224::init(const char *)
// {

// }

bool lx224::openPort(const char *port_name)
{
	closePort();
	return setupPort(port_name, baudrate_);
}

void lx224::closePort()
{
	if(socket_fd_ != -1)
		close(socket_fd_);
	socket_fd_ = -1;
}

bool lx224::setupPort(const char *port_name, int baud)
{
	struct termios newtio;

	socket_fd_ = open(port_name, O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(socket_fd_ < 0)
	{
		printf("[lx224::SetupPort] Error opening serial port!\n");
		return false;
	}

	bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

	newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	// clean the buffer and activate the settings for the port
	tcflush(socket_fd_, TCIFLUSH);
	tcsetattr(socket_fd_, TCSANOW, &newtio);

	tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
	return true;
}

int lx224::readPort(uint8_t *pkt, int length)
{
  return read(socket_fd_, pkt, length);
}

int lx224::writePort(uint8_t *pkt, int length)
{
  return write(socket_fd_, pkt, length);
}

int lx224::flushPort()
{
  return tcflush(socket_fd_,TCIFLUSH);
}