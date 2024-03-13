#include "ttyserial.h"
#include <atomic>
#include <thread>
#include <functional>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <glob.h>

#include <sys/select.h>
#include <signal.h>
#include <stdint.h>

TTYSerialConnection::TTYSerialConnection(const char* dev) {
	m_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (m_fd < 0) {
		perror("open()");
	}

	struct termios tty;
  tcgetattr(m_fd, &tty);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tcsetattr(m_fd, TCSANOW, &tty);
}

TTYSerialConnection::~TTYSerialConnection() {
	if(m_running) {
		Stop();
		m_run_thread.join();
	}
	if(m_fd>0) {
		close(m_fd);
	}
}

void TTYSerialConnection::Run() {
	m_run_thread=std::thread([this]{Loop();});
	m_running=true;
}

void TTYSerialConnection::Loop() {
	uint8_t buffer[1024];
	while(!m_stop) {
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(m_fd, &rfds);
		int ret = select(m_fd+1, &rfds, NULL, NULL, &tv);
		if (ret < 0) {
			perror("select()");
		} else
		if(ret) {
			ret = read(m_fd, buffer, sizeof(buffer)-1);
			if(ret>0) {
				m_callback(buffer, ret, m_callbackArg);
			}
			if(ret<0) {
				printf("SerialConnection::Loop() - error in read\n");
			}
		}
	}
}

int TTYSerialConnection::Write(const uint8_t* data, int len) {
	int bytes_written = write(m_fd, data, len);
	if (bytes_written < 0) {
		perror("write()");
		return -1;
	}
	return bytes_written;
}

ISerialConnection* CreateSerialConnection(const char* name) {
	return new TTYSerialConnection(name);
}

void DestroySerialConnection(ISerialConnection* conn) {
	delete conn;
}
