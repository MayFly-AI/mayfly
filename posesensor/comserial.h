#pragma once
#include "iserial.h"
#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <atomic>
#include <thread>

class COMSerialConnection : public ISerialConnection {
public:
	COMSerialConnection(const char* portName,DWORD baudRate = CBR_9600);
	virtual ~COMSerialConnection() override;
	void Run() override;
	void SetCallback(TDataReadyCallbackFunc f, void* arg) override {
		m_callback=f;
		m_callbackArg=arg;
	}
	void Stop() override {
		m_stop=true;
	}
	int Write(const uint8_t* data, int len) override;

protected:
	void Loop();
	int Read(uint8_t* buffer,unsigned int buf_size);
	bool Valid();

	TDataReadyCallbackFunc m_callback;
	void* m_callbackArg{nullptr};
	HANDLE m_handler;
	bool m_connected;
	COMSTAT m_status;
	DWORD m_errors;
	std::thread m_run_thread;
	std::atomic<bool> m_stop{false};
	bool m_running{false};
};
