#ifndef LX224_H
#define LX224_H

#include <stdio.h>
#include <string.h>
// #include <stdint.h>
#include <cstdint> //uint8_t
#include <termios.h> //Baudrate
#include <linux/serial.h>
#include <unistd.h> //read write
#include <fcntl.h> // O_RDWR|O_NOCTTY|O_NONBLOCK)

#define PKT_HEADER1 0
#define PKT_HEADER2 1
#define PKT_ID 2
#define PKT_LENGTH 3
#define PKT_COMMAND 4
#define PKT_PARAMETER1 5

#define LATENCY_TIMER 16

class lx224
{
	private:
		int socket_fd_;
		int baudrate_;

		double packet_start_time_;
		double packet_timeout_;
		double tx_time_per_byte;

		uint8_t checkSum(uint8_t pkt[]);
		bool setupPort(const char *port_name, int baud);
		int readPort(uint8_t *pkt, int length);
		int writePort(uint8_t *pkt, int length);
		int flushPort();

	public:
		lx224();
		~lx224();
		bool SERVO_MOVE_TIME_WRITE(uint8_t id, int16_t pos, uint16_t t);
		uint16_t SERVO_POS_READ(uint8_t id);
		bool openPort(const char *port_name);
		void closePort();
};

#endif //LX224_H