#include "comserial.h"
#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <psapi.h>
#include <stdio.h>
#include <thread>
#include <chrono>

COMSerialConnection::COMSerialConnection(const char* portName,DWORD baudRate) : m_status() {
	m_errors=0;
	m_connected=false;
	m_handler=CreateFileA(static_cast<LPCSTR>(portName),GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL);
	if(m_handler==INVALID_HANDLE_VALUE) {
		if(GetLastError()==ERROR_FILE_NOT_FOUND) {
			fprintf(stderr, "Failed to open serial port \"%s\"\n", portName);
		}else{
			fprintf(stderr, "Failed to open serial port\n");
		}
	}else{
		DCB dcbSerialParameters={0};
		if(!GetCommState(m_handler,&dcbSerialParameters)) {
			fprintf(stderr, "Failed to get serial parameters\n");
		}else{
			dcbSerialParameters.BaudRate=baudRate;
			dcbSerialParameters.ByteSize=8;
			dcbSerialParameters.StopBits=ONESTOPBIT;
			dcbSerialParameters.Parity=NOPARITY;
			dcbSerialParameters.fDtrControl=DTR_CONTROL_ENABLE;
			if(!SetCommState(m_handler,&dcbSerialParameters)) {
				fprintf(stderr, "Failed to set serial parameters\n");
			}else{
				m_connected=true;
				PurgeComm(m_handler,PURGE_RXCLEAR|PURGE_TXCLEAR);
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			}
		}
	}
}

COMSerialConnection::~COMSerialConnection() {
	if(m_running) {
		Stop();
		m_run_thread.join();
	}
	if(m_connected) {
		m_connected=false;
		CloseHandle(m_handler);
	}
}

int COMSerialConnection::Read(uint8_t* buffer,unsigned int buf_size) {
	DWORD bytesRead{};
	unsigned int toRead=0;
	ClearCommError(m_handler,&m_errors,&m_status);
	if(m_status.cbInQue>0) {
		if(m_status.cbInQue>buf_size) {
			toRead=buf_size;
		}else{
			toRead=m_status.cbInQue;
		}
	}
	memset((void*)buffer,0,buf_size);
	if(ReadFile(m_handler,(void*)buffer,toRead,&bytesRead,NULL)) {
		return bytesRead;
	}
	return 0;
}

int COMSerialConnection::Write(const uint8_t* data, int len) {
	DWORD bytesSend{};
	if(!WriteFile(m_handler,(void*)data,len,&bytesSend,0)) {
		ClearCommError(m_handler,&m_errors,&m_status);
		return -1;
	}
	return bytesSend;
}

bool COMSerialConnection::Valid() {
	if(!ClearCommError(m_handler,&m_errors,&m_status)) {
		m_connected=false;
	}
	return m_connected;
}

void COMSerialConnection::Run() {
	m_run_thread=std::thread([this]{Loop();});
	m_running=true;
}

void COMSerialConnection::Loop() {
	uint8_t buffer[1024];
	while(!m_stop) {
		int ret = Read(buffer, sizeof(buffer));
		if(ret>0) {
			m_callback(buffer, ret, m_callbackArg);
		}
		if(ret<0) {
			printf("COMSerialConnection::Loop() - error in read\n");
		}
	}
}

ISerialConnection* CreateSerialConnection(const char* name) {
	return new COMSerialConnection(name);
}

void DestroySerialConnection(ISerialConnection* conn) {
	delete conn;
}
